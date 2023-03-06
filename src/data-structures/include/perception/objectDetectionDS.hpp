#pragma once
#ifndef OBJECT_DETECTION_DATA_STRUCTURES_HPP
#define OBJECT_DETECTION_DATA_STRUCTURES_HPP

#include "perception_common_headers.hpp"

struct ObjectDetectionData
{
    std::vector<std::vector<cv::Point>> m_detectedCentroids;
    std::vector<std::vector<cv::Point>> m_left_top;
    std::vector<std::vector<float>> m_width;
    std::vector<std::vector<float>> m_height;
    std::vector<std::vector<int16_t>> classIds;
    std::vector<std::vector<float>> confidences;
    std::vector<int> camIdx;
    // std::vector<std::string> currTimeStamps;
};

namespace boost {
namespace serialization {

    // object detection seri
    template <typename Ar>
    void serialize(Ar &ar, ObjectDetectionData &objdat, unsigned)
    {
        ar &objdat.m_detectedCentroids &objdat.m_left_top &objdat.m_width &objdat.m_height &objdat.classIds &objdat.confidences;
    }
}
}
#endif
