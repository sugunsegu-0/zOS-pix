#pragma once
#ifndef LOCALIZTATION_HPP
#define LOCALIZTATION_HPP

#include "localizationDS.hpp"


using namespace std;

struct odom_Header{
    uint32_t seq=0.0;
	double time=0.0;
	string frame_id="";
};

struct Point{
    double x=0.0;
    double y=0.0;
    double z=0.0;
};
struct Pose{
    Point position;
    Quaternion orientation;
};

struct Twist{
    Vector3 linear;
    Vector3 angular;

};

struct PoseWithCovariance{
    Pose pose;
    double covariance[64]; 
};
struct TwistWithCovariance{
    Twist twist;
    double covariance[64]; 
};

struct Odometry{
    odom_Header header;
    string child_frame_id;
    PoseWithCovariance pose;
    TwistWithCovariance twist;
};

struct PoseWithCovarianceStamped{
    odom_Header header;
    PoseWithCovariance pose;
};
struct TwistWithCovarianceStamped{   
    odom_Header header;
    TwistWithCovariance twist;
};

namespace boost {
namespace serialization {
  template <typename Ar> void serialize(Ar& ar, odom_Header& header, unsigned) 
    {
        ar & header.seq;
        ar & header.time;
        ar & header.frame_id;
    }


    template <typename Ar> void serialize(Ar& ar, Point& point, unsigned) 
    {
        ar & point.x;
        ar & point.y;
        ar & point.z;
    }


    template <typename Ar> void serialize(Ar& ar, Pose& pose, unsigned) 
    {
        ar & pose.position;
        ar & pose.orientation;
    }

    template <typename Ar> void serialize(Ar& ar, PoseWithCovariance& pose_cov, unsigned) 
    {
        ar & pose_cov.pose;
        ar & pose_cov.covariance;
    }

    template <typename Ar> void serialize(Ar& ar, Twist& twist, unsigned) 
    {
        ar & twist.linear;
        ar & twist.angular;
    }


    template <typename Ar> void serialize(Ar& ar, TwistWithCovariance& twist_cov, unsigned) 
    {
        ar & twist_cov.twist;
        ar & twist_cov.covariance;
    }


    template <typename Ar> void serialize(Ar& ar, Odometry& odom, unsigned) 
    {
        ar & odom.header;
        ar & odom.child_frame_id;
        ar & odom.pose;
        ar & odom.twist;

    }
}
}
#endif