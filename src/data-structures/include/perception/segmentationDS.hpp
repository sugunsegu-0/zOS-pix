#pragma once
#ifndef SEGMENTATION_DATA_STRUCTURES_HPP
#define SEGMENTATION_DATA_STRUCTURES_HPP

#include "perception_common_headers.hpp"

struct SegmentationData
{
    std::vector<int> camIdx;
    std::vector<float> bufferLane;
    std::vector<float> bufferFreespace;
    int frame_count;
    int width;
    int height;
    int channel;
    int eleSize;
};

namespace boost {
namespace serialization {

template <typename Ar>
void serialize(Ar &ar, SegmentationData &sd, unsigned)
{
    ar &sd.bufferLane &sd.bufferLane &sd.bufferFreespace &sd.frame_count &sd.width &sd.height &sd.channel &sd.eleSize;
}
}
}
#endif