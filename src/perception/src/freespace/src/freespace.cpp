#include "freespace.hpp"

FreeSpace::FreeSpace()
: PerceptionModel<FreeSpaceConfig>()
{
    #ifdef RENDER_PERCEPTION_FREESPACE 
    initialize_render(int(modelConfig.frameActualDim[0]), int(modelConfig.frameActualDim[1]/modelConfig.batchSize));
    stitchedFrame = std::make_shared<cv::Mat>(int(modelConfig.frameActualDim[1]/modelConfig.batchSize), int(modelConfig.frameActualDim[0]), CV_8UC3, cv::Scalar(0, 0, 0));
    #endif
}

FreeSpace::~FreeSpace()
{
    #ifdef RENDER_PERCEPTION_FREESPACE
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
    #endif
}

bool FreeSpace::loadModel()
{
    return true;
}

bool FreeSpace::preprocess(float* buffer)
{
    switch (modelConfig.rec_buf_type)
    {
        case REC_BUF_RGB:
            {
                for(auto camIdx : modelConfig.camIdxToRunOn)
                {
                    cv::Mat frameCPU(cv::Size(modelConfig.frameActualDim[0], modelConfig.frameActualDim[1]),
                                    CV_32FC3, 
                                    buffer + camIdx * modelConfig.frameActualDim[0] * modelConfig.frameActualDim[1] * 3);
                    frameCPU.convertTo(frameCPU, CV_8UC3, 1, 0);
                    cv::Mat grayCPU;
                    cv::cvtColor(frameCPU, grayCPU, cv::COLOR_BGR2GRAY);
                    grayCPU.convertTo(grayCPU, CV_8UC1);
                    outputBatch.push_back(grayCPU);
                }
            }
            break;
        default:
            std::runtime_error("`modelConfig.rec_buf_type` not supported. Supported types for Freespace - REC_BUF_RGB");
            break;
    }
    return true;
}

bool FreeSpace::runInference(float* buffer)
{
    frameFreeSpacePoints.clear();
    frameHierarchy.clear();
    outputBatch.clear();
    preprocess(buffer);
    postprocess();

    #ifdef RENDER_PERCEPTION_FREESPACE
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

bool FreeSpace::postprocess()
{
    for(auto frameEdge : outputBatch)
    {
        cv::threshold(frameEdge, frameEdge, 1, 255, THRESH_OTSU);
        std::vector<std::vector<cv::Point>> contourPoints, filteredContours;
        std::vector<cv::Vec4i> fHierarchy, filteredHierarchy;
        cv::findContours(frameEdge, contourPoints, fHierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        for(int i=0; i<contourPoints.size(); ++i)
        {
            if(cv::contourArea(contourPoints[i]) > modelConfig.contourAreaFilter)
            {
                filteredContours.push_back(contourPoints[i]);
                filteredHierarchy.push_back(fHierarchy[i]);
            }
        }
        if(filteredContours.size() > 0)
        {
            frameFreeSpacePoints.push_back(filteredContours);
            frameHierarchy.push_back(filteredHierarchy);
        }
    }
    return true; 
}

bool FreeSpace::getOutput(FreeSpaceData* freeSpaceData)
{
    freeSpaceData->frameFreeSpacePoints = frameFreeSpacePoints;
    freeSpaceData->camIdx = modelConfig.camIdxToRunOn;
    return true;
}

#ifdef RENDER_PERCEPTION_FREESPACE
bool FreeSpace::renderResults(std::vector<cv::Mat>* frames)
{ 
    cv::Scalar color( 255, 255, 255 );
    for(int i=0; i<frames->size(); ++i)
    {
        for(int j=0; j<frameFreeSpacePoints.at(i).size(); ++j)
        {
            cv::drawContours(frames->at(i), frameFreeSpacePoints.at(i), j, color, 1, LINE_8, frameHierarchy.at(i));
        }
    }
    cv::hconcat(*frames , *(stitchedFrame.get()));
    cv::resize(*(stitchedFrame.get()), *(stitchedFrame.get()), cv::Size(modelConfig.frameActualDim[0], modelConfig.frameActualDim[1]/modelConfig.batchSize));
    return true;
}
#endif