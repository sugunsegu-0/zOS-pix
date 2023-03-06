#pragma once
#ifndef PERCEPTION_TEMPLATE_NODE_HPP
#define PERCEPTION_TEMPLATE_NODE_HPP

#include <iostream>
#include <thread>
#include <future>
// #include <vector>
#include "globals.hpp"
// #include "perception.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "image_transport/image_transport.h"
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// #include "sensor_msgs/image_encodings.hpp"
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include "std_msgs/msg/string.hpp"


class PerceptionModule
{
  public:
    PerceptionModule();
    // std::vector<std_msgs::msg::Header> frameHeaders;
    // std::vector<std::shared_ptr<cv_bridge::CvImagePtr>> Mats_ptr;
    std::vector<cv::Mat> currFrames;
    // std::vector<std::string> currTimeStamps;
    // std::vector<sensor_msgs::msg::Image::SharedPtr> cam_msgs;
    virtual void topic_callback(){};
    
    // bool loopImages();

    // const sensor_msgs::msg::Image::SharedPtr msg

    // bool getMatFromMessage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    // bool batchImages(std::vector<const sensor_msgs::msg::Image::ConstSharedPtr&> msgs);
    // void cam_callback(const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_0,
    //                   const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_1,
    //                   const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_2,
    //                   const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_3,
    //                   const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_4);

    // bool getMatFromMessage(const sensor_msgs::msg::Image::SharedPtr msg, int id);
    virtual bool batchImages(){return true; };
    virtual bool inference(){ return true; };
    // virtual bool convertToICEMsg(){ return true; };
    virtual bool threadedPlotting(){ return true; };
    virtual bool writeIceMsg(){ return true; };

    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // message_filters::Subscriber<sensor_msgs::msg::Image> cam_subcribers[CAMERA_COUNT];
};

#endif