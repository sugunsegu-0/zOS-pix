#pragma once
#ifndef MOTION_PLANNING_DATA_STRUCTURES_HPP
#define MOTION_PLANNING_DATA_STRUCTURES_HPP

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

 
struct path
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> v;
};

namespace boost{
namespace serialization
    {
        template <typename Ar> 
        void serialize(Ar &ar, path &pa, unsigned)
    {
        ar &pa.v;
        ar &pa.x;
        ar &pa.y;
    }
    }
}
#endif