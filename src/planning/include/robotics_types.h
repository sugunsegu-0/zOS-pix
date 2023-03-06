#ifndef _robotics_TYPES_H
#define _robotics_TYPES_H

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace robotics{

using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_f>;

};

#endif
