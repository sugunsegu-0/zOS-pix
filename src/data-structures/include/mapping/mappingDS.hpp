#pragma once
#ifndef MAPPING_DATA_STRUCTURES_HPP
#define MAPPING_DATA_STRUCTURES_HPP

#include <iostream>
#include <vector>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <fstream>
#include <math.h>
#include <typeinfo>
#include <opencv2/opencv.hpp>


struct Obstcales
{
    std::vector<std::vector<cv::Point>> Obs;
    std::vector<int> labels;
    // std::vector<std::string> currTimeStamps;
};

// typedef std::pair<std::vector<cv::Point>, float> laneConfPairVec;
// typedef std::vector<laneConfPairVec> lanesPerFrameVec;

struct Lanes
{
    std::vector<std::vector<std::pair<std::vector<cv::Point>, float>>> lanesperframe_out;
    // std::vector<std::string> currTimeStamps;
};
struct Freespace
{
    std::vector<std::vector<std::vector<cv::Point>>> freespaces_homo;
};

namespace boost {
namespace serialization {

    template <typename Ar>
    void serialize(Ar &ar, Obstcales &obs, unsigned)
    {
        ar &obs.labels;
        ar &obs.Obs;
    }

    template <typename Ar>
    void serialize(Ar &ar, Lanes &lane, const unsigned int version)
    {
        ar &lane.lanesperframe_out;
    }

    template <typename Ar>
    void serialize(Ar &ar, Freespace &free, const unsigned int version)
    {
        ar &free.freespaces_homo;
    }

    // template <class Archive>
    // void serialize(Archive &ar, cv::Point &pt3, const unsigned int)
    // {
    //     ar &pt3.x;
    //     ar &pt3.y;
    // }

    }
}

#endif
