#pragma once
#ifndef COMMONS_HPP
#define COMMONS_HPP

// IPC

#include "IPC/serialize-deserialize.hpp"
#include "boost_serial/boost_serial_template.hpp"

// boost headers
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/utility.hpp>
// #include <opencv2/opencv.hpp>

// namespace boost {
// namespace serialization {

//     template <class Archive>
//     void serialize(Archive &ar, cv::Point &pt3, const unsigned int)
//     {
//         ar &pt3.x;
//         ar &pt3.y;
//     }
// }
// }



#endif