#pragma once
#ifndef LANE_ESTIMATION_DATA_STRUCTURES_HPP
#define LANE_ESTIMATION_DATA_STRUCTURES_HPP

#include "perception_common_headers.hpp"

typedef std::pair<std::vector<cv::Point>, float> laneConfPairVec;
typedef std::vector<laneConfPairVec> lanesPerFrameVec;

struct LaneEstimationData
{
    std::vector<lanesPerFrameVec> detectedLanes;
    std::vector<std::vector<int>> detectedLaneTypes;
    std::vector<int> camIdx;
    // std::vector<std::string> currTimeStamps;
};

namespace boost {
namespace serialization {

    template <typename Ar>
    void serialize(Ar &ar, LaneEstimationData &led, unsigned)
    {
        ar &led.detectedLanes &led.detectedLaneTypes &led.camIdx;
    }

    template <typename Ar>
    void serialize(Ar &ar, laneConfPairVec &pr, const unsigned int version)
    {
        ar &pr.first;
        ar &pr.second;
    }
    }
}

#endif
