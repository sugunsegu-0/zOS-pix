#include "object-detection.hpp"

ObjectDetection::ObjectDetection(const char* m_modelFile, bool m_isEngine, bool m_isOnnx)
: PerceptionModel<ObjectDetectionConfig>(m_modelFile, m_isEngine, m_isOnnx)
{
    #ifdef RENDER_PERCEPTION_OBJECT_DETECTION 
    initialize_render(int(modelConfig.frameActualDim[0]), int(modelConfig.frameActualDim[1]/modelConfig.batchSize));
    stitchedFrame = std::make_shared<cv::Mat>(int(modelConfig.frameActualDim[1]/modelConfig.batchSize), int(modelConfig.frameActualDim[0]), CV_8UC3, cv::Scalar(0, 0, 0));
    #endif
}

ObjectDetection::~ObjectDetection()
{
    #ifdef RENDER_PERCEPTION_OBJECT_DETECTION 
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
    #endif
}

bool ObjectDetection::loadModel()
{
    PerceptionModel<ObjectDetectionConfig>::loadModel();
    initialize_labelBoxMap();

    classIds.reserve(modelConfig.camIdxToRunOn.size());
    labels.reserve(modelConfig.camIdxToRunOn.size());
    confidences.reserve(modelConfig.camIdxToRunOn.size());
    m_left_top.reserhconcatve(modelConfig.camIdxToRunOn.size());
    m_width.reserve(modelConfig.camIdxToRunOn.size());
    m_height.reserve(modelConfig.camIdxToRunOn.size());
    m_detectedCentroids.reserve(modelConfig.camIdxToRunOn.size());
    m_detectedBoxList.reserve(modelConfig.camIdxToRunOn.size());
    m_detectedBoxListFloat.reserve(modelConfig.camIdxToRunOn.size());

    for(int i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        classIds.push_back(std::vector<int16_t>());
        labels.push_back(std::vector<const char*>());
        confidences.push_back(std::vector<float>());
        m_left_top.push_back(std::vector<cv::Point>());
        m_width.push_back(std::vector<float>());
        m_height.push_back(std::vector<float>());
        m_detectedCentroids.push_back(std::vector<cv::Point>());
        m_detectedBoxList.push_back(std::vector<cv::Rect2i>());
        m_detectedBoxListFloat.push_back(std::vector<cv::Rect2f>());
    }

    NV_CUDA_CHECK(cudaMalloc(&receivedBufferGpuUYVY,
                  sizeof(uint8_t) * modelConfig.batchSize * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 2));
    NV_CUDA_CHECK(cudaMalloc(&receivedBufferGpuRGB,
                  sizeof(uint8_t) * modelConfig.batchSize * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 3));

    #ifdef RENDER_PERCEPTION_OBJECT_DETECTION
    NV_CUDA_CHECK(cudaMallocHost(&receivedBufferCpuRGB,
                  sizeof(uint8_t) * modelConfig.batchSize * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 3));
    #endif

    return true;
}

bool ObjectDetection::preprocess(uint8_t* buffer)
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
                        &ObjectDetection::convertBufferUYVY2RGB,
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
                    hconcatmodelConfig.batchSize,
                    modelConfig.frameActualDim[0],
                    modelConfig.frameActualDim[1],
                    modelConfig.inputChannelCount,
                    modelConfig.camIdxToRunOn
                );
            }
            break;
        default:
            std::runtime_error("`modelConfig.rec_buf_type` not supported. Supported types for Object Detection- REC_BUF_UYVY, REC_BUF_RGB");
            break;
    }

    
    imgBatchInPipeline = imgBatchInPipeline.contiguous();
    return true;
}

bool ObjectDetection::runInference(uint8_t* buffer)
{
    preprocess(buffer);
    std::vector<void*> inputs = {imgBatchInPipeline.data<float>()};
    PerceptionModel<ObjectDetectionConfig>::runInference(&inputs);
    postprocess();

    #ifdef RENDER_PERCEPTION_OBJECT_DETECTION
    tmpMats.clear();
    switch (modelConfig.rec_buf_type)
    {
        case REC_BUF_UYVY:
            {
                NV_CUDA_CHECK(cudaMemcpy(
                    receivedBufferCpuRGB,
                    receivedBufferGpuRGB,
                    modelConfig.batchSize * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 3 * sizeof(uint8_t),
                    cudaMemcpyDeviceToHost
                ));
                for(int i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
                {
                    cv::Mat frame(cv::Size(modelConfig.frameActualDim[0], modelConfig.frameActualDim[1]), CV_8UC3,
                                receivedBufferCpuRGB + (i * modelConfig.frameActualDim[1] * modelConfig.frameActualDim[0] * 3));
                    cv::cvtColor(frame, frame, CV_BGR2RGB);
                    tmpMats.push_back(frame);
                }
            }
            break;
        case REC_BUF_RGB:
            {
                for(int i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
                {
                    cv::Mat frame(cv::Size(modelConfig.frameActualDim[0], modelConfig.frameActualDim[1]), CV_8UC3,
                                  buffer + (i * modelConfig.frameActualDim[1] * modelConfig.frameActualDim[0] * 3));
                    tmpMats.push_back(frame);
                }
            }
            break;
        default:
            std::runtime_error("`modelConfig.rec_buf_type` not supported. Supported types for Object Detection- REC_BUF_UYVY, REC_BUF_RGB");
            break;
    }
    renderResults(&tmpMats);
    #endif

    return true;
}

bool ObjectDetection::postprocess()
{
    for(auto id : outputIdxs)
    {
        NV_CUDA_CHECK(cudaMemcpy(m_HostBuffers.at(id),
                                m_DeviceBuffers.at(id),
                                getSizeByDim(ioDims.at(id)) * sizeof(float),
                                cudaMemcpyDeviceToHost));

    }
    float* out = (float*)m_HostBuffers.at(outputIdxs.at(0));
    interpretOutputPointerwithROI(out);
    return true; 
}

bool ObjectDetection::getOutput(ObjectDetectionData* objData)
{
    objData->m_detectedCentroids = m_detectedCentroids;
    objData->m_left_top = m_left_top;
    objData->m_width = m_width;
    objData->m_height = m_height;
    objData->classIds = classIds;
    objData->confidences = confidences;
    objData->camIdx = modelConfig.camIdxToRunOn;
    return true;
}

#ifdef RENDER_PERCEPTION_OBJECT_DETECTION
bool ObjectDetection::renderResults(std::vector<cv::Mat>* frames)
{
    int thickness = 1;
    for(uint32_t i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    {
        cv::resize(frames->at(i), frames->at(i), cv::Size(modelConfig.frameActualDim[0], modelConfig.frameActualDim[1]));
        for(auto box: m_detectedBoxListFloat.at(i))
        {
            cv::Point p1(box.x, box.y);
            cv::Point p2(box.x + box.width, box.y + box.height);
            cv::rectangle(frames->at(i), p1, p2,
                            Scalar(255, 0, 0),
                            thickness, LINE_8);
        }
    }
    cv::hconcat(*frames, *(stitchedFrame.get()));
    cv::resize(*(stitchedFrame.get()), *(stitchedFrame.get()), cv::Size(int(modelConfig.frameActualDim[0]), int(modelConfig.frameActualDim[1]/modelConfig.batchSize)));
    return true; 
}
#endif

void ObjectDetection::initialize_labelBoxMap()
{
    for (uint32_t i=0; i < sizeof(modelConfig.lane_n)/sizeof(modelConfig.lane_n[0]); ++i)
    {
        labelName.insert(std::pair<int, const char*>(modelConfig.lane_n[i], modelConfig.label_n[i]));
    }
}

bool ObjectDetection::interpretOutputPointerwithROI(float* out)
{
    for(uint32_t i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    // for(uint32_t i=0; i<1; ++i)
    {
        postProcessingThreads[i] = std::thread(&ObjectDetection::outputPostProcessing,
                                                this,
                                                out,
                                                i);
    }
    for(uint32_t i=0; i<modelConfig.camIdxToRunOn.size(); ++i)
    // for(uint32_t i=0; i<1; ++i)
    {
        postProcessingThreads[i].join();
    }

    return true;
}

// bool ObjectDetection::outputPostProcessing(const float* out, int frameId)
// {
//     uint32_t numBBoxes = 0U;
//     const float *data = out;
//     float prob_sum = 0;
//     int g_count = 0;

//     // Per frame outputs
//     std::vector<int16_t> fclassIds;
//     std::vector<const char*> flabels;
//     std::vector<float> fconfidences;
//     std::vector<cv::Rect> boxes;
//     std::vector<cv::Rect2f> frame_detectedBoxListFloat;
//     std::vector<cv::Rect2i> frame_detectedBoxList;
//     std::vector<cv::Point> frame_centroid;
//     std::vector<cv::Point> frame_left_top;
//     std::vector<float> frame_width;
//     std::vector<float> frame_height;

//     // Intermediate procecssing mediums
//     std::vector<int16_t> clsIdxs;
//     std::vector<float> confs;

//     auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);
//     auto outTensor = torch::from_blob(
//         (float*)out + (frameId * modelConfig.outAnchorBoxes * modelConfig.outCols),
//         { 
//             modelConfig.outputBlobDims[0], 
//             modelConfig.outputBlobDims[1]
//         },
//         options     
//     );

//     if (modelConfig.permuteOutput)
//         outTensor = outTensor.permute({1, 0});

//     if (modelConfig.confidenceIdxInOut != -1)
//     {max_class_score
//         auto maskTensor = outTensor.index({at::indexing::Slice(), modelConfig.confidenceIdxInOut}) >= modelConfig.COVERAGE_THRESHOLD;
//         auto indicesTensor = maskTensor.argwhere().flatten();
//         outTensor = outTensor.index_select(0, indicesTensor).to(torch::kCUDA);
//     }
    
//     auto bboxTensor = outTensor.index({at::indexing::Slice(), 
//                                        at::indexing::Slice(modelConfig.bboxIdxStart, modelConfig.bboxIdxStart + 4)});
//     auto classTensor = outTensor.index({at::indexing::Slice(),
//                                         at::indexing::Slice(modelConfig.classPredictionStartIdxInOut, 
//                                                             modelConfig.classPredictionStartIdxInOut + modelConfig.numOfClasses)}); 

    
//     return true;
// }

bool ObjectDetection::outputPostProcessing(float* out, int frameId)
{
    uint32_t numBBoxes = 0U;
    const float *data = out;
    float prob_sum = 0;
    int g_count = 0;

    // Per frame outputs
    std::vector<int16_t> fclassIds;
    std::vector<const char*> flabels;
    std::vector<float> fconfidences; 
    std::vector<cv::Rect> boxes;
    std::vector<cv::Rect2f> frame_detectedBoxListFloat;
    std::vector<cv::Rect2i> frame_detectedBoxList;
    std::vector<cv::Point> frame_centroid;
    std::vector<cv::Point> frame_left_top;
    std::vector<float> frame_width;
    std::vector<float> frame_height;

    // Intermediate procecssing mediums
    std::vector<int16_t> clsIdxs;
    std::vector<float> confs;

    const float* frame_data;    

    if (modelConfig.permuteOutput)
    {
        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        auto outTensor = torch::from_blob(
            (float*)out + (frameId * modelConfig.outAnchorBoxes * modelConfig.outCols),
            { 
                modelConfig.outputBlobDims[0], 
                modelConfig.outputBlobDims[1]
            },
            options     
        );
        outTensor = outTensor.permute({1, 0});
        frame_data = outTensor.contiguous().data<float>();
    }
    else
    {
        frame_data = out + frameId * modelConfig.outputBlobDims[0] * modelConfig.outputBlobDims[1];
    }

    for (int i = 0; i < modelConfig.outAnchorBoxes; ++i)
    {   
    
        // Discard bad detections and continue
        if (modelConfig.confidenceIdxInOut == -1 || frame_data[modelConfig.confidenceIdxInOut] >= modelConfig.COVERAGE_THRESHOLD)
        {
            float confidence = 1.0;
            if(modelConfig.confidenceIdxInOut != -1)
                confidence = frame_data[modelConfig.confidenceIdxInOut];

            const float * classes_scores = frame_data + modelConfig.classPredictionStartIdxInOut;

            // Create a 1x85 Mat and store class scores of 80 classes.

            std::vector<float> scores;
            
            // Perform minMaxLoc and acquire the index of best class  score.
            
            double max_class_score;
            for (int i = 0; i < modelConfig.numOfClasses; i++) 
            {
                scores.push_back(*(classes_scores+i));
            }
            
            int max_id=0;
            for (int i = 0; i < modelConfig.numOfClasses; i++) 
            {
                if(scores[i]>scores[max_id])
                {
                    max_id=i;
                }
            }
            max_class_score=scores[max_id];
            // Continue if the class score is above the threshold.
            if (max_class_score > modelConfig.NMS_SCORE_THRESHOLD)
            {
                // Store class ID and confidence in the pre-defined respective vectors.
                confs.push_back(confidence);
                clsIdxs.push_back(max_id);

                if(modelConfig.outType == OUT_CCHW)
                {
                    // Center
                    float cx = frame_data[0];  // c
                    float cy = frame_data[1];  // c
                    // Box dimension.
                    float w = frame_data[2];   // w
                    float h = frame_data[3];   // h
                    int left = int((cx - 0.5 * w) * modelConfig.x_factor);
                    int top = int((cy - 0.5 * h) * modelConfig.y_factor);
                    int width = int(w * modelConfig.x_factor);
                    int height = int(h * modelConfig.y_factor);
                    boxes.push_back(cv::Rect2f(left, top, width, height));
                }
                else if(modelConfig.outType == OUT_XYXY)
                {
                    // top left
                    float tx = frame_data[0];   // x
                    float ty = frame_data[1];   // y
                    // bottom right
                    float bx = frame_data[2];   // x
                    float by = frame_data[3];   // y
                    int left = int(tx * modelConfig.x_factor);
                    int top = int(ty * modelConfig.y_factor);
                    int width = int((bx - tx) * modelConfig.x_factor);
                    int height = int((ty- by) * modelConfig.y_factor);
                    boxes.push_back(cv::Rect2f(left, top, width, height));
                }
                else
                {
                    std::runtime_error("modelConfig.outType not supported. Supported Types are - OUT_CCHW, OUT_XYXY");
                }
            }
        }
        frame_data += modelConfig.outCols;
        // Jump to the next row.
    }
    std::vector<int> indices;

    NMSBoxes(boxes, confs,
            modelConfig.NMS_SCORE_THRESHOLD,
            modelConfig.NMS_IOU_THRESHOLD,
            indices, 
            modelConfig.NMS_ETA, 
            modelConfig.topK_nms);

    for (uint i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        cv::Rect2f box = boxes[idx];
        float left = box.x;
        float top = box.y;
        float width = box.width;
        float height = box.height;
        cv::Rect2f bboxFloat {left, top, width, height};
        cv::Rect2i bbox;
        bbox.width  = static_cast<int32_t>(std::round(width));
        bbox.height = static_cast<int32_t>(std::round(height));
        bbox.x      = static_cast<int32_t>(std::round(left));
        bbox.y      = static_cast<int32_t>(std::round(top));
        cv::Point bottomCentroid{static_cast<float>(bbox.x) + (static_cast<float>(bbox.width/2)), static_cast<float>(bbox.y) + static_cast<float>(bbox.height)};
        fclassIds.push_back(clsIdxs[idx]);
        fconfidences.push_back(confs[idx]);
        flabels.push_back(labelName.at(clsIdxs[idx]));
        frame_centroid.push_back(bottomCentroid);
        frame_detectedBoxList.push_back(bbox);
        frame_detectedBoxListFloat.push_back(bboxFloat);
        frame_left_top.push_back(cv::Point(bbox.x, bbox.y));
        frame_width.push_back(bbox.width);
        frame_height.push_back(bbox.height);
        numBBoxes++;
    }
    m_left_top.at(frameId) = frame_left_top;
    m_width.at(frameId) = frame_width;
    m_height.at(frameId) = frame_height;
    m_detectedCentroids.at(frameId) = frame_centroid;
    m_detectedBoxList.at(frameId) = frame_detectedBoxList;
    m_detectedBoxListFloat.at(frameId) = frame_detectedBoxListFloat;
    classIds.at(frameId) = fclassIds;
    labels.at(frameId) = flabels;
    confidences.at(frameId) = fconfidences;
    return true;
}

// std::vector<cv::Rect> ObjectDetection::update_boxes(std::vector<cv::Rect> m_detectedBoxList_u, std::vector<int16_t> label_l)
// {
//     for(uint16_t i=0;i<m_detectedBoxList_u.size();i++)
//     {
//         if(label_l[i]==3 || label_l[i]==1)
//         {
//             for(uint16_t j=0;j<m_detectedBoxList_u.size();j++)
//             {
//                 if(label_l[j]==0)
//                 {
//                     float iou_val=iou(m_detectedBoxList_u[i].x,
//                                         m_detectedBoxList_u[i].y, 
//                                         m_detectedBoxList[i].x + m_detectedBoxList_u[i].width, 
//                                         m_detectedBoxList_u[i].y + m_detectedBoxList_u[i].height, 
//                                         m_detectedBoxList_u[j].x, 
//                                         m_detectedBoxList_u[j].y, 
//                                         m_detectedBoxList_u[j].x + m_detectedBoxList_u[j].width, 
//                                         m_detectedBoxList_u[j].y + m_detectedBoxList_u[j].height);
//                     if (iou_val>0.01)
//                     {
//                         cv::Rect2f bbox;
//                         bbox.x = m_detectedBoxList_u[j].x;
//                         bbox.y = m_detectedBoxList_u[j].y;
//                         bbox.width = m_detectedBoxList_u[j].width;
//                         bbox.height= m_detectedBoxList_u[i].height + m_detectedBoxList_u[j].height;
//                         m_detectedBoxList_u.push_back(bbox);
//                         label_l.push_back(10);
//                         label_l.erase(std::next(label_l.begin(), j));
//                         m_detectedBoxList_u.erase(std::next(m_detectedBoxList_u.begin(), j));
//                         label_l.erase(std::next(label_l.begin(), i));
//                         m_detectedBoxList_u.erase(std::next(m_detectedBoxList_u.begin(), i));
//                         i--;
//                         break;
//                     }
//                 }
//             }
//         }        
//     }
    
//     return m_detectedBoxList_u;
// }

// float ObjectDetection::iou(float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4)
// {
//     float x_max=std::max(x1,x3);
//     float x_min=std::min(x2,x4);
//     float y_max=std::max(y1,y3);
//     float y_min=std::min(y2,y4);
//     float width = abs(x_max-x_min);
//     float hieght = abs(y_max-y_min);
//     float area = width * hieght;
//     float wid1 = abs(x2-x1);
//     float hie1 = abs(y2-y1);
//     float wid2 = abs(x4-x3);
//     float hei2 = abs(y4-y3);
//     float area1 = wid1 *hie1;
//     float area2 = wid2 * hei2;
//     float area_un = area1 + area2 - area;
//     float iou = area / area_un;
//     return iou;
// }