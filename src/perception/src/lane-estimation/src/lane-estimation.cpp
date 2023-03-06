#include "lane-estimation.hpp"

LaneEstimation::LaneEstimation(const char* m_modelFile, bool m_isEngine, bool m_isOnnx)
: PerceptionModel<LaneEstimationConfig>(m_modelFile, m_isEngine, m_isOnnx)
{
    #ifdef RENDER_PERCEPTION_LANE_ESTIMATION 
    initialize_render(int(modelConfig.frameActualDim[0]), int(modelConfig.frameActualDim[1]/modelConfig.batchSize));
    stitchedFrame = std::make_shared<cv::Mat>(int(modelConfig.frameActualDim[1]/modelConfig.batchSize), int(modelConfig.frameActualDim[0]), CV_8UC3, cv::Scalar(0, 0, 0));
    #endif
}

LaneEstimation::~LaneEstimation()
{
    #ifdef RENDER_PERCEPTION_LANE_ESTIMATION 
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
    #endif
}

bool LaneEstimation::loadModel()
{
    PerceptionModel<LaneEstimationConfig>::loadModel();

    float pointGap = modelConfig.inputBlobDims[1]/pointsPerLine;
    float y=modelConfig.inputBlobDims[1];
    for (uint32_t j=0; j<pointsPerLine; ++j, y=y+pointGap)
    {
        t_detectedLanes[j].x = -2.0f;
        t_detectedLanes[j].y = (modelConfig.inputBlobDims[1] - (j * pointGap)) /  modelConfig.inputBlobDims[1];
    }

    for(uint32_t i=0; i<modelConfig.batchSize; ++i)
    {
        lanesPerFrameVec lanesInit;
        detectedLanes.push_back(lanesInit);

        std::vector<int> laneTypesInit; 
        detectedLaneTypes.push_back(laneTypesInit);
    }

    return true;
}

bool LaneEstimation::preprocess(float* buffer)
{
    switch (modelConfig.rec_buf_type)
    {
        case REC_BUF_RGB:
            {
                BufferToNormalizedGPUTensor(
                    buffer,
                    imgBatchInPipeline,
                    modelConfig.batchSize,
                    modelConfig.frameActualDim[0],
                    modelConfig.frameActualDim[1],
                    modelConfig.inputChannelCount,
                    modelConfig.camIdxToRunOn,
                    BUF_FLOAT32
                );
            }
            break;
        default:
            std::runtime_error("`modelConfig.rec_buf_type` not supported. Supported types for Lane Estimation - REC_BUF_RGB");
            break;
    }

    imgBatchInPipeline = imgBatchInPipeline.contiguous();
    return true;
}

bool LaneEstimation::runInference(float* buffer)
{ 
    preprocess(buffer);
    std::vector<void*> inputs = {imgBatchInPipeline.data<float>()};
    PerceptionModel<LaneEstimationConfig>::runInference(&inputs);
    postprocess();

    #ifdef RENDER_PERCEPTION_LANE_ESTIMATION
    tmpMats.clear();
    for(int i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        cv::Mat frame(cv::Size(modelConfig.frameActualDim[0], modelConfig.frameActualDim[1]), CV_32FC3,
                      buffer + (i * modelConfig.frameActualDim[1] * modelConfig.frameActualDim[0] * 3));
        frame.convertTo(frame, CV_8UC3, 1, 0);
        tmpMats.push_back(frame);
    }
    renderResults(&tmpMats);
    #endif

    return true; 
}

bool LaneEstimation::postprocess()
{ 
    for(auto id : outputIdxs)
    {
        NV_CUDA_CHECK(cudaMemcpy(m_HostBuffers.at(id),
                                m_DeviceBuffers.at(id),
                                getSizeByDim(ioDims.at(id)) * sizeof(float),
                                cudaMemcpyDeviceToHost));
    }
    
    const float* logitsOut = (const float*)m_HostBuffers.at(m_engine->getBindingIndex("logits"));
    const float* curvesOut = (const float*)m_HostBuffers.at(m_engine->getBindingIndex("curves"));
    interpretOutputPointer(logitsOut, curvesOut);
    return true; 
}

bool LaneEstimation::getOutput(LaneEstimationData* lanesEstData)
{
    lanesEstData->detectedLanes = detectedLanes;
    lanesEstData->detectedLaneTypes = detectedLaneTypes;
    lanesEstData->camIdx = modelConfig.camIdxToRunOn;
    return true;
}

#ifdef RENDER_PERCEPTION_LANE_ESTIMATION
bool LaneEstimation::renderResults(std::vector<cv::Mat>* frames)
{ 
    for(uint32_t camIdx; camIdx < frames->size(); ++camIdx)
    {
        lanesPerFrameVec lanesCam = detectedLanes.at(camIdx);
        uint32_t loopLimit;
        if (m_maxDetections >= lanesCam.size())
        {
            loopLimit = lanesCam.size();
        }
        else
        {
            loopLimit = m_maxDetections;
        }
        for(uint32_t i=0; i<loopLimit; ++i)
        {
            laneConfPairVec* lane = &(lanesCam.at(i));
            if (lane->second >= modelConfig.PRESENCE_THRESHOLD && lane->first.size() > 1)
            {
                const cv::Point *pts = (const cv::Point*) cv::Mat(lane->first).data;
                int npts = cv::Mat(lane->first).rows;

                polylines(frames->at(camIdx), &pts, &npts, 1, false, Scalar(0, 255, 0));
            }
        }
    }
    cv::hconcat(*frames , *(stitchedFrame.get()));
    cv::resize(*(stitchedFrame.get()), *(stitchedFrame.get()), cv::Size(modelConfig.frameActualDim[0], modelConfig.frameActualDim[1]/modelConfig.batchSize));
    return true;
}
#endif

void LaneEstimation::softmax(float* input, size_t size)
{
    assert(0 <= size <= sizeof(input) / sizeof(float));

    float m, sum, constant;

    m = -INFINITY;
    for (uint32_t i = 0; i < size; ++i)
    {
        if (m < input[i])
            m = input[i];
    }

    sum = 0.0;
    for (uint32_t i = 0; i < size; ++i) {
        sum += exp(input[i] - m);
    }

    constant = m + log(sum);
    for (uint32_t i = 0; i < size; ++i) {
        input[i] = exp(input[i] - constant);
    }
}

std::vector<cv::Point> LaneEstimation::lanePointCalc(cv::Point2f(&lanePoints)[pointsPerLine], int* laneType,
                                                    float k, float f, float m, float n, float b1, float b2,
                                                    float lowerBoundY, float upperBoundY)
{
    std::vector<cv::Point> lanePointsFiltered;
    for(uint32_t i=0; i<pointsPerLine; ++i)
    {
        float y = lanePoints[i].y;
        if (y >= lowerBoundY && y <= upperBoundY)
        {
            cv::Point p;
            p.x = (( k / pow(y - f, 2.0) ) + ( m / (y - f) ) + n + (b1 * y) - b2) * modelConfig.inputBlobDims[0];
            p.y = y * modelConfig.inputBlobDims[1];
            lanePointsFiltered.push_back(p);
        }
    }
    if(lanePointsFiltered.size() > 0 && lanePointsFiltered.at(0).x < (int)modelConfig.inputBlobDims[0]/2)
    {
        *(laneType) = 0;
    }
    else if(lanePointsFiltered.size() > 0 && lanePointsFiltered.at(0).x >= (int)modelConfig.inputBlobDims[0]/2)
    {
        *(laneType) = 1;
    }
    else
    {
        *(laneType) = -1;
    }
    return lanePointsFiltered;
}

void LaneEstimation::interpretOutputPointer(const float* logitsOut, const float* curvesOut)
{
    for(uint32_t i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        detectedLanes.at(i).clear();
        detectedLaneTypes.at(i).clear();
        sortedPresence[i].reset(new uint32_t[7]);
    }
    detectedLanes.clear();
    detectedLaneTypes.clear();

    for(uint32_t i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        lanesPerFrameVec lanesInit;
        detectedLanes.push_back(lanesInit);

        std::vector<int> laneTypesInit; 
        detectedLaneTypes.push_back(laneTypesInit);
    }

    auto logitsBufferSizePerFrame = getSizeByDim(ioDims.at(m_engine->getBindingIndex("logits")))/modelConfig.batchSize;
    auto curvesBufferSizePerFrame = getSizeByDim(ioDims.at(m_engine->getBindingIndex("curves")))/modelConfig.batchSize;

    for(uint32_t i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        postProcessingThreads[i] = std::thread(&LaneEstimation::postprocessOutputPerFrame,
                                                this,
                                                &(detectedLanes.at(i)),
                                                &(detectedLaneTypes.at(i)),
                                                sortedPresence[i].get(),  
                                                logitsOut + i*logitsBufferSizePerFrame,
                                                logitsBufferSizePerFrame,
                                                curvesOut + i*curvesBufferSizePerFrame,
                                                curvesBufferSizePerFrame,
                                                i);
    }

    for(uint32_t i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        postProcessingThreads[i].join();
    }
}

void LaneEstimation::postprocessOutputPerFrame( lanesPerFrameVec* detectedLanesPerFrame,
                                                std::vector<int>* detectedLaneTypesPerFrame,
                                                uint32_t* sortedPresencePerFrame,
                                                const float* logitsOut, uint32_t logitsBufferSizePerFrame,
                                                const float* curvesOut, uint32_t curvesBufferSizePerFrame, 
                                                uint32_t cameraIdx)
{
    std::vector<double> logitsBuffer;
    for(uint32_t i=0; i < logitsBufferSizePerFrame; i++)
    {
        logitsBuffer.push_back(*(logitsOut + i));
    }
    std::vector<std::size_t> logitsShape = {7, 2};
    auto reshapedLogitsBuffer = xt::adapt(logitsBuffer, logitsShape);
    std::vector<std::size_t> shape = {7};
    xt::xarray<double> lanePresence(shape);
    for (uint32_t i=0 ; i<7; ++i)
    {
        float laneLogits[2] = {reshapedLogitsBuffer(i, 0), reshapedLogitsBuffer(i, 1)};
        softmax(&laneLogits[0], sizeof(laneLogits)/sizeof(laneLogits[0]));
        lanePresence(i) = laneLogits[1];
    }

    auto xtSortedPresence = xt::argsort(lanePresence);
    for (uint32_t i=0; i<7; ++i)
    {
        *(sortedPresencePerFrame + i) = xtSortedPresence(i);
    }

    std::vector<double> curvesBuffer;
    for(uint32_t i=0;i < curvesBufferSizePerFrame; i++)
    {
        curvesBuffer.push_back(*(curvesOut + i));
    }
    std::vector<std::size_t> curvesShape = {7, 8};
    auto reshapedCurvesBuffer = xt::adapt(curvesBuffer, curvesShape);
    auto boundsView = xt::view(reshapedCurvesBuffer, xt::all(), xt::range(0, 2));
    auto curvesView = xt::view(reshapedCurvesBuffer, xt::all(), xt::range(2, 8));

    for (int i=6; i>=static_cast<int>(7-m_maxDetections); i--)
    {
        int laneType;
        auto laneIdx = *(sortedPresencePerFrame + i);
        if (lanePresence(laneIdx) >= modelConfig.PRESENCE_THRESHOLD)
        {
            laneConfPairVec* filteredLaneVec = new laneConfPairVec;
            auto filteredLanePoints = lanePointCalc(t_detectedLanes,
                                                    &laneType,
                                                    curvesView(laneIdx, 0), curvesView(laneIdx, 1), curvesView(laneIdx, 2),
                                                    curvesView(laneIdx, 3), curvesView(laneIdx, 4), curvesView(laneIdx, 5), boundsView(laneIdx, 0),
                                                    boundsView(laneIdx, 1));
            filteredLaneVec->first = filteredLanePoints;
            filteredLaneVec->second = lanePresence(laneIdx);
            (*detectedLanesPerFrame).push_back(*filteredLaneVec);
        }
        (*detectedLaneTypesPerFrame).push_back(laneType);
    }
}