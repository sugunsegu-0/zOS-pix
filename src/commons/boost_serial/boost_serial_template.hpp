// #pragma once
// #ifndef BOOST_SERIAL_TEMPLATE_HPP
// #define BOOST_SERIAL_TEMPLATE_HPP


// namespace boost {
// namespace serialization {
/*------------Control DS----------*/
// #include "control/control-DS.hpp"
/*------------Control DS----------*/


// namespace boost {
// namespace serialization {

// // /*------------------------------template common for all----------------------------------------*/


// //     // template <class Archive>
// //     // void serialize(Archive &ar, cv::Point &pt3, const unsigned int)
// //     // {
// //     //     ar &pt3.x;
// //     //     ar &pt3.y;
// //     // }

// // /*-------------------------------Perception-----------------------------------------------------*/

// // // freespace data template
    
// //     template <typename Ar>
// //     void serialize(Ar &ar, FreeSpaceData &fsd, unsigned)
// //     {
// //         ar &fsd.frameFreeSpacePoints &fsd.camIdx;
// //     }

// //     template <class Archive>
// //     void serialize(Archive &ar, cv::Point3_<float> &pt3, const unsigned int)
// //     {
// //         ar &pt3.x;
// //         ar &pt3.y;
// //         ar &pt3.z;
// //     }

// // // lane-estimation data template

// //     template <typename Ar>
// //     void serialize(Ar &ar, LaneEstimationData &led, unsigned)
// //     {
// //         ar &led.detectedLanes &led.detectedLaneTypes &led.camIdx;
// //     }

// //     template <typename Ar>
// //     void serialize(Ar &ar, laneConfPairVec &pr, const unsigned int version)
// //     {
// //         ar &pr.first;
// //         ar &pr.second;
// //     }

// // // object-detection data template

// //     template <typename Ar>
// //     void serialize(Ar &ar, ObjectDetectionData &objdat, unsigned)
// //     {
// //         ar &objdat.m_detectedCentroids &objdat.m_left_top &objdat.m_width &objdat.m_height &objdat.classIds &objdat.confidences;
// //     }

// // // segmentation data template

// //     template <typename Ar>
// //     void serialize(Ar &ar, SegmentationData &sd, unsigned)
// //     {
// //         ar &sd.bufferLane &sd.bufferLane &sd.bufferFreespace &sd.frame_count &sd.width &sd.height &sd.channel &sd.eleSize;
// //     }

// // /*-------------------------------Perception-----------------------------------------------------*/

// // /*-------------------------------Mapping-----------------------------------------------------*/

// //     template <typename Ar>
// //     void serialize(Ar &ar, Obstcales &obs, unsigned)
// //     {
// //         ar &obs.labels;
// //         ar &obs.Obs;
// //     }

// //     template <typename Ar>
// //     void serialize(Ar &ar, Lanes &lane, const unsigned int version)
// //     {
// //         ar &lane.lanesperframe_out;
// //     }

// //     template <typename Ar>
// //     void serialize(Ar &ar, Freespace &free, const unsigned int version)
// //     {
// //         ar &free.freespaces_homo;
// //     }

// // /*-------------------------------Mapping-----------------------------------------------------*/

// // /*-------------------------------Localization-----------------------------------------------------*/

// // // odometry
// //     template <typename Ar> void serialize(Ar& ar, odom_Header& header, unsigned) 
// //     {
// //         ar & header.seq;
// //         ar & header.time;
// //         ar & header.frame_id;
// //     }


// //     template <typename Ar> void serialize(Ar& ar, Point& point, unsigned) 
// //     {
// //         ar & point.x;
// //         ar & point.y;
// //         ar & point.z;
// //     }


// //     template <typename Ar> void serialize(Ar& ar, Pose& pose, unsigned) 
// //     {
// //         ar & pose.position;
// //         ar & pose.orientation;
// //     }

// //     template <typename Ar> void serialize(Ar& ar, PoseWithCovariance& pose_cov, unsigned) 
// //     {
// //         ar & pose_cov.pose;
// //         ar & pose_cov.covariance;
// //     }

// //     template <typename Ar> void serialize(Ar& ar, Twist& twist, unsigned) 
// //     {
// //         ar & twist.linear;
// //         ar & twist.angular;
// //     }


// //     template <typename Ar> void serialize(Ar& ar, TwistWithCovariance& twist_cov, unsigned) 
// //     {
// //         ar & twist_cov.twist;
// //         ar & twist_cov.covariance;
// //     }


// //     template <typename Ar> void serialize(Ar& ar, Odometry& odom, unsigned) 
// //     {
// //         ar & odom.header;
// //         ar & odom.child_frame_id;
// //         ar & odom.pose;
// //         ar & odom.twist;

// //     }

// // // imu
// //     template <typename Ar> void serialize(Ar& ar, Header& header, unsigned) 
// //     {
// //         ar & header.time;
// //         ar & header.frame_id;
// //     }

// //     template <typename Ar> void serialize(Ar& ar, Quaternion& quaternion, unsigned)
// //     {
// //         ar & quaternion.w;
// //         ar & quaternion.x;
// //         ar & quaternion.y;
// //         ar & quaternion.z;
// //     }

// //     template <typename Ar> void serialize(Ar& ar, Vector3& angular_velocity, unsigned)
// //     {
// //         ar & angular_velocity.x;
// //         ar & angular_velocity.y;
// //         ar & angular_velocity.z;
// //     }

// //     template <typename Ar> void serialize(Ar& ar, Imu& imu, unsigned)
// //     {
// //         ar & imu.angular_velocity;
// //         ar & imu.angular_velocity_covariance;
// //         ar & imu.header;
// //         ar & imu.linear_acceleration;
// //         ar & imu.linear_acceleration_covariance;
// //         ar & imu.orientation;
// //         ar & imu.orientation_covariance;
// //     }

// // // gnss
// //     template <typename Ar> void serialize(Ar& ar, GPSStatus& status, unsigned) 
// //     {
// //     	// ar & status.STATUS_NO_FIX;
// //     	// ar & status.STATUS_FIX;
// //     	ar & status.STATUS_SBAS_FIX;
// //     	ar & status.STATUS_GBAS_FIX;
// //     	// ar & status.STATUS_DGPS_FIX;
// //     	ar & status.STATUS_WAAS_FIX;
// //     	ar & status.SOURCE_NONE;
// //     	ar & status.SOURCE_GPS;
// //     	ar & status.SOURCE_POINTS;
// //     	ar & status.SOURCE_DOPPLER;
// //     	ar & status.SOURCE_ALTIMETER;
// //     	ar & status.SOURCE_MAGNETIC;
// //     	ar & status.SOURCE_GYRO;
// //     	ar & status.SOURCE_ACCEL;
// //     	ar & status.header;
// //     	ar & status.satellites_used;
// //     	ar & status.satellites_visible;
// //     	ar & status.status;
// //     	ar & status.motion_source;
// //     	ar & status.orientation_source;
// //     	ar & status.position_source;
// //     }

// //     template <typename Ar> void serialize(Ar& ar, GPSFix& fix, unsigned) 
// //     {
// //     	ar & fix.COVARIANCE_TYPE_UNKNOWN;
// //     	ar & fix.COVARIANCE_TYPE_APPROXIMATED;
// //     	ar & fix.COVARIANCE_TYPE_DIAGONAL_KNOWN;
// //     	ar & fix.COVARIANCE_TYPE_KNOWN;
// //     	ar & fix.header;
// //     	ar & fix.status;
// //     	ar & fix.latitude;
// //     	ar & fix.longitude;
// //     	ar & fix.altitude;
// //     	ar & fix.track;
// //     	ar & fix.speed;
// //     	ar & fix.climb;
// //     	ar & fix.pitch;
// //     	ar & fix.roll;
// //     	ar & fix.dip;
// //     	ar & fix.time;
// //     	ar & fix.gdop;
// //     	ar & fix.pdop;
// //     	ar & fix.hdop;
// //     	ar & fix.vdop;
// //     	ar & fix.tdop;
// //     	ar & fix.err;
// //     	ar & fix.err_horz;
// //     	ar & fix.err_vert;
// //     	ar & fix.err_track;
// //     	ar & fix.err_speed;
// //     	ar & fix.err_climb;
// //     	ar & fix.err_time;
// //     	ar & fix.err_pitch;
// //     	ar & fix.err_roll;
// //     	ar & fix.err_dip;
// //     	ar & fix.position_covariance;
// //     	ar & fix.position_covariance_type;
// //     }

// // /*-------------------------------Localization-----------------------------------------------------*/

// // /*---------------------------------Planning-----------------------------------------------------*/
// //     template <typename Ar>
// //     void serialize(Ar &ar, path &pa, unsigned)
// //     {
// //         ar &pa.v;
// //         ar &pa.x;
// //         ar &pa.y;
// //     }

// // /*---------------------------------Planning-----------------------------------------------------*/

// // }
// // }
//     template <typename Ar>
//     void serialize(Ar &ar, FreeSpaceData &fsd, unsigned)
//     {
//         ar &fsd.frameFreeSpacePoints &fsd.camIdx;
//     }

//     template <class Archive>
//     void serialize(Archive &ar, cv::Point3_<float> &pt3, const unsigned int)
//     {
//         ar &pt3.x;
//         ar &pt3.y;
//         ar &pt3.z;
//     }

// // lane-estimation data template

//     template <typename Ar>
//     void serialize(Ar &ar, LaneEstimationData &led, unsigned)
//     {
//         ar &led.detectedLanes &led.detectedLaneTypes &led.camIdx;
//     }

//     template <typename Ar>
//     void serialize(Ar &ar, laneConfPairVec &pr, const unsigned int version)
//     {
//         ar &pr.first;
//         ar &pr.second;
//     }

// // object-detection data template

//     template <typename Ar>
//     void serialize(Ar &ar, ObjectDetectionData &objdat, unsigned)
//     {
//         ar &objdat.m_detectedCentroids &objdat.m_left_top &objdat.m_width &objdat.m_height &objdat.classIds &objdat.confidences;
//     }

// // segmentation data template

//     template <typename Ar>
//     void serialize(Ar &ar, SegmentationData &sd, unsigned)
//     {
//         ar &sd.bufferLane &sd.bufferLane &sd.bufferFreespace &sd.frame_count &sd.width &sd.height &sd.channel &sd.eleSize;
//     }

// /*-------------------------------Perception-----------------------------------------------------*/

// /*-------------------------------Mapping-----------------------------------------------------*/

//     template <typename Ar>
//     void serialize(Ar &ar, Obstcales &obs, unsigned)
//     {
//         ar &obs.labels;
//         ar &obs.Obs;
//     }

//     template <typename Ar>
//     void serialize(Ar &ar, Lanes &lane, const unsigned int version)
//     {
//         ar &lane.lanesperframe_out;
//     }

//     template <typename Ar>
//     void serialize(Ar &ar, Freespace &free, const unsigned int version)
//     {
//         ar &free.freespaces_homo;
//     }

// /*-------------------------------Mapping-----------------------------------------------------*/

// /*-------------------------------Localization-----------------------------------------------------*/

// // odometry
//     template <typename Ar> void serialize(Ar& ar, odom_Header& header, unsigned) 
//     {
//         ar & header.seq;
//         ar & header.time;
//         ar & header.frame_id;
//     }


//     template <typename Ar> void serialize(Ar& ar, Point& point, unsigned) 
//     {
//         ar & point.x;
//         ar & point.y;
//         ar & point.z;
//     }


//     template <typename Ar> void serialize(Ar& ar, Pose& pose, unsigned) 
//     {
//         ar & pose.position;
//         ar & pose.orientation;
//     }

//     template <typename Ar> void serialize(Ar& ar, PoseWithCovariance& pose_cov, unsigned) 
//     {
//         ar & pose_cov.pose;
//         ar & pose_cov.covariance;
//     }

//     template <typename Ar> void serialize(Ar& ar, Twist& twist, unsigned) 
//     {
//         ar & twist.linear;
//         ar & twist.angular;
//     }


//     template <typename Ar> void serialize(Ar& ar, TwistWithCovariance& twist_cov, unsigned) 
//     {
//         ar & twist_cov.twist;
//         ar & twist_cov.covariance;
//     }


//     template <typename Ar> void serialize(Ar& ar, Odometry& odom, unsigned) 
//     {
//         ar & odom.header;
//         ar & odom.child_frame_id;
//         ar & odom.pose;
//         ar & odom.twist;

//     }

// // imu
//     template <typename Ar> void serialize(Ar& ar, Header& header, unsigned) 
//     {
//         ar & header.time;
//         ar & header.frame_id;
//     }

//     template <typename Ar> void serialize(Ar& ar, Quaternion& quaternion, unsigned)
//     {
//         ar & quaternion.w;
//         ar & quaternion.x;
//         ar & quaternion.y;
//         ar & quaternion.z;
//     }

//     template <typename Ar> void serialize(Ar& ar, Vector3& angular_velocity, unsigned)
//     {
//         ar & angular_velocity.x;
//         ar & angular_velocity.y;
//         ar & angular_velocity.z;
//     }

//     template <typename Ar> void serialize(Ar& ar, Imu& imu, unsigned)
//     {
//         ar & imu.angular_velocity;
//         ar & imu.angular_velocity_covariance;
//         ar & imu.header;
//         ar & imu.linear_acceleration;
//         ar & imu.linear_acceleration_covariance;
//         ar & imu.orientation;
//         ar & imu.orientation_covariance;
//     }

// // gnss
//     template <typename Ar> void serialize(Ar& ar, GPSStatus& status, unsigned) 
//     {
//     	// ar & status.STATUS_NO_FIX;
//     	// ar & status.STATUS_FIX;
//     	ar & status.STATUS_SBAS_FIX;
//     	ar & status.STATUS_GBAS_FIX;
//     	// ar & status.STATUS_DGPS_FIX;
//     	ar & status.STATUS_WAAS_FIX;
//     	ar & status.SOURCE_NONE;
//     	ar & status.SOURCE_GPS;
//     	ar & status.SOURCE_POINTS;
//     	ar & status.SOURCE_DOPPLER;
//     	ar & status.SOURCE_ALTIMETER;
//     	ar & status.SOURCE_MAGNETIC;
//     	ar & status.SOURCE_GYRO;
//     	ar & status.SOURCE_ACCEL;
//     	ar & status.header;
//     	ar & status.satellites_used;
//     	ar & status.satellites_visible;
//     	ar & status.status;
//     	ar & status.motion_source;
//     	ar & status.orientation_source;
//     	ar & status.position_source;
//     }

//     template <typename Ar> void serialize(Ar& ar, GPSFix& fix, unsigned) 
//     {
//     	ar & fix.COVARIANCE_TYPE_UNKNOWN;
//     	ar & fix.COVARIANCE_TYPE_APPROXIMATED;
//     	ar & fix.COVARIANCE_TYPE_DIAGONAL_KNOWN;
//     	ar & fix.COVARIANCE_TYPE_KNOWN;
//     	ar & fix.header;
//     	ar & fix.status;
//     	ar & fix.latitude;
//     	ar & fix.longitude;
//     	ar & fix.altitude;
//     	ar & fix.track;
//     	ar & fix.speed;
//     	ar & fix.climb;
//     	ar & fix.pitch;
//     	ar & fix.roll;
//     	ar & fix.dip;
//     	ar & fix.time;
//     	ar & fix.gdop;
//     	ar & fix.pdop;
//     	ar & fix.hdop;
//     	ar & fix.vdop;
//     	ar & fix.tdop;
//     	ar & fix.err;
//     	ar & fix.err_horz;
//     	ar & fix.err_vert;
//     	ar & fix.err_track;
//     	ar & fix.err_speed;
//     	ar & fix.err_climb;
//     	ar & fix.err_time;
//     	ar & fix.err_pitch;
//     	ar & fix.err_roll;
//     	ar & fix.err_dip;
//     	ar & fix.position_covariance;
//     	ar & fix.position_covariance_type;
//     }

// /*-------------------------------Localization-----------------------------------------------------*/

// /*---------------------------------Planning-----------------------------------------------------*/
//     template <typename Ar>
//     void serialize(Ar &ar, path &pa, unsigned)
//     {
//         ar &pa.v;
//         ar &pa.x;
//         ar &pa.y;
//     }

// /*---------------------------------Planning-----------------------------------------------------*/
// /*---------------------------------Planning-----------------------------------------------------*/
//     template <typename Ar>
//     void serialize(Ar &ar, ctrl &control, unsigned)
//     {
//         ar &control.linear_v;
//         ar &control.steer;
        
//     }

// /*---------------------------------Planning-----------------------------------------------------*/

// }
// }

// #endif