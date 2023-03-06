#include "node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PerceptionModule::PerceptionModule()
{
    // rclcpp::QoS qos(10);
    // auto rmw_qos_profile = qos.get_rmw_qos_profile();
    
    // Mats_ptr.reserve(5);
    currFrames.reserve(5);
    // frameHeaders.reserve(5);
    for(uint32_t i=0; i<5; ++i)
    {
        // Mats_ptr.push_back(std::make_shared<cv_bridge::CvImagePtr>(cv_bridge::CvImagePtr()));
        currFrames.push_back(cv::Mat());

        // WARNING: Header is not being added anymore

        //frameHeaders.push_back(std_msgs::msg::Header());            
    }
}

// void PerceptionModule::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
// {
//     // RCLCPP_INFO(this->get_logger(),"I heard!");
//     loopImages(msg);
//     inference();
//     convertToROSMsg();
//     threadedPlotting();
//     writeROSMsg();
//     // RCLCPP_INFO(this->get_logger(),"Output published!");

// }

// bool PerceptionModule::loopImages()
// {
    // const sensor_msgs::msg::Image::SharedPtr msg
    // cam_msgs.clear();
    // cam_msgs = {msg, msg, msg, msg, msg};
    // for(int i=0; i<5; ++i)
    // {
    //     getMatFromMessage(cam_msgs.at(i), i);
    // }
    // batchImages();
    // return true;
// }

// void PerceptionModule::cam_callback(
//     const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_0,
//     const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_1,
//     const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_2,
//     const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_3,
//     const sensor_msgs::msg::Image::ConstSharedPtr& cam_msg_4)
// {
//     std::vector<const sensor_msgs::msg::Image::ConstSharedPtr&> cam_msgs{&cam_msg_0, &cam_msg_1, &cam_msg_2, &cam_msg_3, &cam_msg_4};
//     for(int i=0; i<CAMERA_COUNT; ++i)
//     {
//         getMatFromMessage(cam_msgs.at(i));
//     }
//     batchImages(cam_msgs);
// }

// bool PerceptionModule::getMatFromMessage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
// {
//     auto MatPosition = std::stoi(msg->header.frame_id);
//     *(Mats_ptr[MatPosition]) = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     return true;
// }

// bool PerceptionModule::batchImages(std::vector<const sensor_msgs::msg::Image::ConstSharedPtr&> msgs)
// {
//     for(int i=0; i<CAMERA_COUNT; ++i)
//     {
//         auto MatPosition = std::stoi(msgs.at(i)->header.frame_id);
//         currFrames.at(MatPosition) = (*Mats_ptr[MatPosition])->image;
//     }
//     return true;
// }

// bool PerceptionModule::getMatFromMessage(const sensor_msgs::msg::Image::SharedPtr msg, int id)
// {
//     // auto MatPosition = std::stoi(msg->header.frame_id);
//     *(Mats_ptr[id]) = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     return true;
// }