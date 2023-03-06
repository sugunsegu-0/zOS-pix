#pragma once
#ifndef PERCEPTION_HPP
#define PERCEPTION_HPP

#include "dlRF.hpp"
#include "perception/perceptionDS.hpp"
#include "object-detection.hpp"
#include "segmentation.hpp"
// #include "mask-correction.hpp"
#include "lane-estimation.hpp"
// #include "freespace.hpp"

struct PerceptionData
{
    ObjectDetectionData objectDetectionData;
    LaneEstimationData laneEstimationData;
    // FreeSpaceData freeSpaceData;
};

// struct PerceptionConfig
// {
//     std::vector<int> camIdxForObjDet = {0, 1, 2, 3, 4};
//     std::vector<int> camIdxForLaneEst = {0, 1, 2, 3, 4};
//     std::vector<int> camIdxForFreeSpace = {0, 1, 2, 3, 4};
// };

class Perception  //: public AVModule
{

    public:

        Perception();

        ~Perception();

        bool onInitialize();

        bool onProcess(std::vector<cv::Mat> imgBatch);

        bool getOutput(PerceptionData* perceptionOutputCurrent);

        // bool onPublishData();

        // bool onPublishDataWithTimeStamp(std::vector<std::string> currTimeStamps);

        // bool onRender(std::vector<cv::Mat> imgBatch);

        // bool onRelease();

    private:
        std::unique_ptr<ObjectDetection> objDet;
        std::unique_ptr<Segmentation> segMen;
        std::unique_ptr<LaneEstimation> laneEst;
        // std::unique_ptr<FreeSpace> freespace;
        // PerceptionConfig perceptionConfig;
        
        // bool onInitialize() override;

        // bool onProcess() override;

        // bool onRender();

        // bool onRelease() override;
        
};

#endif