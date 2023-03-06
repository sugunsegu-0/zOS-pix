#pragma once
#ifndef CONTROL_STRUCTURES_HPP
#define CONTROL_STRUCTURES_HPP

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
#include <ctime>
#include <chrono>
 
struct ctrl
{
    float linear_v=0.0;
    float steer=0.0;
    double time=0.0;
    
};
template <typename Ar>
    void serialize(Ar &ar, ctrl &control, unsigned)
    {
        ar &control.linear_v;
        ar &control.steer;
        ar &control.time;
    }

#endif