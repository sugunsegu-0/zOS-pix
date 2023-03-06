#pragma once
#ifndef IMU_STRUCTURES_HPP
#define IMU_STRUCTURES_HPP

#include<iostream>

struct Header{
	double time=0.0;
	std::string frame_id="";
};

struct Quaternion{
	double x=0.0;
	double y=0.0;
	double z=0.0;
	double w=1.0;
};

struct Vector3{
	double x=0.0;
	double y=0.0;
	double z=0.0;
};

struct Imu{
	Header header;
	Quaternion orientation;
	double orientation_covariance[9];
	Vector3 angular_velocity;
	double angular_velocity_covariance[9];
	Vector3 linear_acceleration;
	double linear_acceleration_covariance[9];
};
namespace boost {
namespace serialization {
	 template <typename Ar> void serialize(Ar& ar, Header& header, unsigned) 
    {
        ar & header.time;
        ar & header.frame_id;
    }

    template <typename Ar> void serialize(Ar& ar, Quaternion& quaternion, unsigned)
    {
        ar & quaternion.w;
        ar & quaternion.x;
        ar & quaternion.y;
        ar & quaternion.z;
    }

    template <typename Ar> void serialize(Ar& ar, Vector3& angular_velocity, unsigned)
    {
        ar & angular_velocity.x;
        ar & angular_velocity.y;
        ar & angular_velocity.z;
    }

    template <typename Ar> void serialize(Ar& ar, Imu& imu, unsigned)
    {
        ar & imu.angular_velocity;
        ar & imu.angular_velocity_covariance;
        ar & imu.header;
        ar & imu.linear_acceleration;
        ar & imu.linear_acceleration_covariance;
        ar & imu.orientation;
        ar & imu.orientation_covariance;
    }
}
}
#endif