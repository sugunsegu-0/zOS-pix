#include "segmentation.hpp"

Segmentation::Segmentation(const char* m_modelFile, bool m_isEngine, bool m_isOnnx)
: PerceptionModel<SegmentationConfig>(m_modelFile, m_isEngine, m_isOnnx)
{
    for(int i=0; i<3; ++i)
    {
        cudaStreamCreate(&m_cudaStream_datatransfers[i]);
        
    }

    NV_CUDA_CHECK(cudaMalloc(&receivedBufferGpuUYVY,
                  sizeof(uint8_t) * modelConfig.batchSize * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 2));
    NV_CUDA_CHECK(cudaMalloc(&receivedBufferGpuRGB,
                  sizeof(uint8_t) * modelConfig.batchSize * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 3));

    #ifdef RENDER_PERCEPTION_SEGMENTATION
    initialize_render(int(modelConfig.outputBlobDims[0]), int(modelConfig.outputBlobDims[1]));
    stitchedFrame = std::make_shared<cv::Mat>(cv::Size(modelConfig.outputBlobDims[0], modelConfig.outputBlobDims[1]), CV_8UC3, cv::Scalar(0, 0, 0));
    #endif
}

Segmentation::~Segmentation()
{
    for(int i=0; i<3; ++i)
    {
        if(m_cudaStream_datatransfers[i])
        {
            cudaStreamDestroy(m_cudaStream_datatransfers[i]);
        }
    }
    #ifdef RENDER_PERCEPTION_SEGMENTATION
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
    #endif
}

bool Segmentation::loadModel()
{
    PerceptionModel<SegmentationConfig>::loadModel();
    allocatePostProcessingBuffers();
    return true;
}

bool Segmentation::preprocess(uint8_t* buffer)
{
    switch (modelConfig.rec_buf_type)
    {
        case REC_BUF_UYVY:
            {
                NV_CUDA_CHECK(cudaMemcpy(
                    receivedBufferGpuUYVY,
                    buffer,
                    modelConfig.batchSize * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 2 * sizeof(uint8_t),
                    cudaMemcpyHostToDevice
                ));

                std::thread preprocessingThreads[modelConfig.batchSize] = {};
                for(int i=0; i<modelConfig.batchSize; ++i)
                {
                    preprocessingThreads[i] = std::thread(
                        &Segmentation::convertBufferUYVY2RGB,
                        this,
                        (Npp8u*)receivedBufferGpuUYVY + i * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 2,
                        (Npp8u*)receivedBufferGpuRGB + i * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 3
                    );
                }
                for(int i=0; i<modelConfig.batchSize; ++i)
                {
                    preprocessingThreads[i].join();
                }
                NV_CUDA_CHECK(cudaDeviceSynchronize());
                BufferToNormalizedGPUTensor(
                    receivedBufferGpuRGB,
                    imgBatchInPipeline,
                    modelConfig.batchSize,
                    modelConfig.frameActualDim[0],
                    modelConfig.frameActualDim[1],
                    modelConfig.inputChannelCount,
                    modelConfig.camIdxToRunOn,
                    BUF_UINT8,
                    false,
                    true
                );
            }
            break;
        case REC_BUF_RGB:
            {
                BufferToNormalizedGPUTensor(
                    buffer,
                    imgBatchInPipeline,
                    modelConfig.batchSize,
                    modelConfig.frameActualDim[0],
                    modelConfig.frameActualDim[1],
                    modelConfig.inputChannelCount,
                    modelConfig.camIdxToRunOn
                );
            }
            break;
        default:
            std::runtime_error("`modelConfig.rec_buf_type` not supported. Supported types for Segmentation - REC_BUF_UYVY, REC_BUF_RGB");
            break;
    }

    imgBatchInPipeline = imgBatchInPipeline.contiguous();
    return true;
}

bool Segmentation::runInference(uint8_t* buffer)
{
    preprocess(buffer);
    std::vector<void*> inputs = {imgBatchInPipeline.data<float>()};
    PerceptionModel<SegmentationConfig>::runInference(&inputs);
    postprocess();

    #ifdef RENDER_PERCEPTION_SEGMENTATION
    blendedFrameBatch.clear();
    driveableRegionBatch.clear();
    renderResults();
    #endif
    
    return true;
}

bool Segmentation::postprocess()
{
    transferDataToGPU();
    const float* out = (const float*)m_DeviceBuffers.at(outputIdxs.at(0));
    batchedDecodeLabel( maskGPU, 
                        maskLaneGPU,
                        maskFreeSpaceGPU,
                        (float*)m_DeviceBuffers.at(inputIdxs.at(0)),
                        out, 
                        modelConfig.batchSize,
                        modelConfig.inputBlobDims[0], 
                        modelConfig.inputBlobDims[1],
                        meansGPU,
                        stdDevsGPU);
    transferDataToCPU();
    return true; 
}

bool Segmentation::getOutput(SegmentationData* segmentationData)
{
    cudaDeviceSynchronize();

    size_t segOutputEleCount = modelConfig.camIdxToRunOn.size() * modelConfig.outputBlobDims[0] * modelConfig.outputBlobDims[1] * 3;
    processingThreads[0] = std::thread([=](){
                                            segmentationData->bufferFreespace.clear();
                                            segmentationData->bufferFreespace.reserve(segOutputEleCount);
                                            segmentationData->bufferFreespace.insert(
                                                segmentationData->bufferFreespace.end(),
                                                maskFreeSpaceCPU,
                                                maskFreeSpaceCPU + segOutputEleCount
                                            );
                                        });
    processingThreads[1] = std::thread([=](){
                                            segmentationData->bufferLane.clear();
                                            segmentationData->bufferLane.reserve(segOutputEleCount);
                                            segmentationData->bufferLane.insert(
                                                segmentationData->bufferLane.end(), 
                                                maskLaneCPU, 
                                                maskLaneCPU + segOutputEleCount
                                            );
                                        });

    segmentationData->camIdx = modelConfig.camIdxToRunOn;
    segmentationData->width = modelConfig.outputBlobDims[0];
    segmentationData->height = modelConfig.outputBlobDims[1];
    segmentationData->eleSize = sizeof(float);
    segmentationData->frame_count = modelConfig.camIdxToRunOn.size();
    segmentationData->channel = 3;

    processingThreads[0].join();
    processingThreads[1].join();

    return true;
}

bool Segmentation::allocatePostProcessingBuffers()
{
    auto memorySizeToAllocate = modelConfig.batchSize * modelConfig.outputBlobDims[0] * modelConfig.outputBlobDims[1] * 3 * sizeof(float);
    
    // GPU Buffers
    NV_CUDA_CHECK(cudaMalloc((void**)&maskGPU, memorySizeToAllocate));

    NV_CUDA_CHECK(cudaMalloc((void**)&maskLaneGPU, memorySizeToAllocate));

    NV_CUDA_CHECK(cudaMalloc((void**)&maskFreeSpaceGPU, memorySizeToAllocate));

    NV_CUDA_CHECK(cudaMalloc((void**)&meansGPU, 3 * sizeof(float)));

    NV_CUDA_CHECK(cudaMalloc((void**)&stdDevsGPU, 3 * sizeof(float)));

    // CPU Buffers
    NV_CUDA_CHECK(cudaMallocHost((void**)&maskCPU, memorySizeToAllocate));
    NV_CUDA_CHECK(cudaMallocHost((void**)&maskLaneCPU, memorySizeToAllocate));
    NV_CUDA_CHECK(cudaMallocHost((void**)&maskFreeSpaceCPU, memorySizeToAllocate));
    return true;
}

bool Segmentation::transferDataToGPU()
{
    float means[3] = {modelConfig.mean[0], modelConfig.mean[1], modelConfig.mean[2]};
    NV_CUDA_CHECK(cudaMemcpyAsync(
        meansGPU,
        &means,
        3 * sizeof(float),
        cudaMemcpyHostToDevice,
        m_cudaStream_datatransfers[0]));

    float stdDevs[3] = {modelConfig.stdDev[0], modelConfig.stdDev[1], modelConfig.stdDev[2]};
    NV_CUDA_CHECK(cudaMemcpyAsync(
        stdDevsGPU,
        &stdDevs,
        3 * sizeof(float),
        cudaMemcpyHostToDevice,
        m_cudaStream_datatransfers[1]));

    return true;
}

bool Segmentation::transferDataToCPU()
{
    auto memorySizeToAllocate =  modelConfig.batchSize * modelConfig.outputBlobDims[0] * modelConfig.outputBlobDims[1] * 3 * sizeof(float);
    NV_CUDA_CHECK(cudaMemcpyAsync(
        maskCPU,
        maskGPU,
        memorySizeToAllocate,
        cudaMemcpyDeviceToHost,
        m_cudaStream_datatransfers[0]));

    NV_CUDA_CHECK(cudaMemcpyAsync(
        maskLaneCPU,
        maskLaneGPU,
        memorySizeToAllocate,
        cudaMemcpyDeviceToHost,
        m_cudaStream_datatransfers[1]));

    NV_CUDA_CHECK(cudaMemcpyAsync(
        maskFreeSpaceCPU,
        maskFreeSpaceGPU,
        memorySizeToAllocate,
        cudaMemcpyDeviceToHost,
        m_cudaStream_datatransfers[2]));
    return true;
}

#ifdef RENDER_PERCEPTION_SEGMENTATION
bool Segmentation::renderResults()
{
    
    // for(auto id : inputIdxs)
    // {
    //     NV_CUDA_CHECK(cudaMemcpy(m_HostBuffers.at(id),
    //                             m_DeviceBuffers.at(id),
    //                             getSizeByDim(ioDims.at(id)) * sizeof(float),
    //                             cudaMemcpyDeviceToHost));

    // }
    // auto inTensor = torch::from_blob(m_HostBuffers.at(0), {modelConfig.camIdxToRunOn.size(), 3, modelConfig.inputBlobDims[1], modelConfig.inputBlobDims[0]}, torch::kFloat);
    // inTensor = inTensor.permute({0, 2, 3, 1}).contiguous();
    // for(int i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    // {
    //     cv::Mat frame(cv::Size(modelConfig.inputBlobDims[0], modelConfig.inputBlobDims[1]), CV_32FC3,
    //                   inTensor.data<float>() + i * modelConfig.inputBlobDims[1] * modelConfig.inputBlobDims[0] * 3);
    //     cv::cvtColor(frame, frame, CV_BGR2RGB);
    //     blendedFrameBatch.push_back(frame);
    //     cv::imshow("inp"+std::to_string(i), frame);
    // }

    cv::Mat blendedStitchedFrames, driveableStitchedFrames;

    for(int i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        cv::Mat frame(cv::Size(modelConfig.outputBlobDims[0], modelConfig.outputBlobDims[1]), CV_32FC3,
                      maskLaneCPU + i * modelConfig.outputBlobDims[1] * modelConfig.outputBlobDims[0] * 3);
        cv::cvtColor(frame, frame, CV_BGR2RGB);
        frame.convertTo(frame, CV_8UC3, 1, 0);
        blendedFrameBatch.push_back(frame);
    }
    cv::hconcat(blendedFrameBatch , blendedStitchedFrames);
    
    for(int i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        cv::Mat frame(cv::Size(modelConfig.outputBlobDims[0], modelConfig.outputBlobDims[1]), CV_32FC3,
                      maskFreeSpaceCPU + i * modelConfig.outputBlobDims[1] * modelConfig.outputBlobDims[0] * 3);
        cv::cvtColor(frame, frame, CV_BGR2RGB);
        frame.convertTo(frame, CV_8UC3, 1, 0);
        driveableRegionBatch.push_back(frame);
    }
    cv::hconcat(driveableRegionBatch , driveableStitchedFrames);
    
    cv::Mat stitchedArr[] = {blendedStitchedFrames, driveableStitchedFrames};

    cv::vconcat(stitchedArr, 2, *(stitchedFrame.get()));
    cv::resize(*(stitchedFrame.get()), *(stitchedFrame.get()), cv::Size(modelConfig.outputBlobDims[0], modelConfig.outputBlobDims[1]) );

    return true;
}
#endif