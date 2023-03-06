#pragma once
#ifndef FREESPACE_DATA_STRUCTURES_HPP
#define FREESPACE_DATA_STRUCTURES_HPP

#include "perception_common_headers.hpp"

struct FreeSpaceData
{
    std::vector<std::vector<std::vector<cv::Point>>> frameFreeSpacePoints;
    std::vector<int> camIdx;
};

namespace boost {
namespace serialization {

    template <typename Ar>
    void serialize(Ar &ar, FreeSpaceData &fsd, unsigned)
    {
        ar &fsd.frameFreeSpacePoints &fsd.camIdx;
    }

    template <class Archive>
    void serialize(Archive &ar, cv::Point3_<float> &pt3, const unsigned int)
    {
        ar &pt3.x;
        ar &pt3.y;
        ar &pt3.z;
    }
}
}
#endif
