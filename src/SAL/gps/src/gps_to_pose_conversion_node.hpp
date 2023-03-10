
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

#include <geodetic_conv.hpp>

#include <localization/localizationDS.hpp>

#include "commons.hpp"

#include <bits/stdc++.h>
#include <thread>
#include <iostream>
using namespace std;

bool g_is_sim;
bool g_publish_pose;

geodetic_converter::GeodeticConverter g_geodetic_converter;
bool g_got_imu;

// eCAL::CPublisher gps_odom_pub;
 eCAL::CPublisher gps_odom_pub("gps/odom", "std::string");

bool g_trust_gps;
double g_covariance_position_x;
double g_covariance_position_y;
double g_covariance_position_z;
double g_covariance_orientation_x;
double g_covariance_orientation_y;
double g_covariance_orientation_z;
string g_frame_id;
string g_tf_child_frame_id;

double g_lat_ref;
double g_lon_ref;
double g_alt_ref;
int g_count = 1;
bool gps_ref_is_init = false;
int g_its;


bool waypoint_collector = false;
list<double> path;
double prev_x=0.0;
double prev_y=0.0;
double yaw=0.0;
int way_count = 0;

ofstream path_file("/home/mzjet/work/zOS-pix/src/SAL/gps/src/path.txt", std::ofstream::out);


enum EMode
{
  MODE_AVERAGE = 0,
  MODE_WAIT
};
// average over, or wait for, n GPS fixes
EMode g_mode;


void set_gps_reference(GPSFix fix)
{ 
  
  if (!gps_ref_is_init){

    if (fix.status.status < 2){
      std::cout << "No GPS fix" << std::endl;
      return;
    }

    g_lat_ref += fix.latitude;
    g_lon_ref += fix.longitude;
    g_alt_ref += fix.altitude;

    // cout << "Current measurement: %3.8f, %3.8f, %4.2f", msg->latitude, msg->longitude, msg->altitude << endl;

    if (g_count == g_its) {
      if (g_mode == MODE_AVERAGE) {
        g_lat_ref /= g_its;
        g_lon_ref /= g_its;
        g_alt_ref /= g_its;

      } else {
        g_lat_ref = fix.latitude;
        g_lon_ref = fix.longitude;
        g_alt_ref = fix.altitude;
      }

      gps_ref_is_init = true;


      // cout << "Final reference position: << %3.8f << ", " %3.8f << "," << %4.2f << "," <<  g_lat_ref << "," << g_lon_ref << "," << g_alt_ref << endl;
      cout << "Final reference position:" <<  g_lat_ref << ", " <<  g_lon_ref << ", " << g_alt_ref << endl;
      g_lat_ref = 12.90719570;
      g_lon_ref = 77.65235580;
      g_alt_ref = 823.896;
      g_geodetic_converter.initialiseReference(g_lat_ref, g_lon_ref, g_alt_ref);
      
      return;
    } 
  g_count++;

  }
  
}




Odometry odom_msg;
Odometry gps_callback(GPSFix fix)
{ 
  if (!g_got_imu) {
    cout << "No IMU data yet" << endl;
    return odom_msg;
  }

  if (fix.status.status < 2) {
    cout << "No GPS fix" << endl;
    return odom_msg;
  }

  if (!g_geodetic_converter.isInitialised()) {
    cout << "No GPS reference point set, not publishing"  << endl;
    return odom_msg;
  }

  double x, y, z;
  g_geodetic_converter.geodetic2Enu(fix.latitude, fix.longitude, fix.altitude, &x, &y, &z);

  // cout << "x: " << x << "y: " << y << endl;  
  if (waypoint_collector){
    
    if (sqrt(pow(x - prev_x, 2) + pow(y - prev_y, 2)) > 0.5){
      
      path.push_back(x);
      path.push_back(y);
      prev_x = x;
      prev_y = y;
      cout << "Waypoint[" << way_count << "] - X: " << x << ", Y: " << y << ", Yaw: " << yaw << endl;
      way_count = way_count +1;

    }
  } 


  // Fill up pose message
  
  odom_msg.header.seq = 0;
  odom_msg.header.time = fix.header.time;
  odom_msg.header.frame_id = g_frame_id;
  odom_msg.child_frame_id = g_tf_child_frame_id;
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = z;

  odom_msg.pose.covariance[0] = fix.position_covariance[0];
  odom_msg.pose.covariance[4] = fix.position_covariance[4];
  odom_msg.pose.covariance[8] = fix.position_covariance[8];

  // odom_msg.pose.pose.orientation = 0

  odom_msg.twist.twist.linear.x = x;
  odom_msg.twist.twist.linear.y = y;
  odom_msg.twist.twist.linear.z = z;
  // odom_msg.twist.twist.angular = 0

  odom_msg.twist.covariance[0] = fix.position_covariance[0];
  odom_msg.twist.covariance[4] = fix.position_covariance[4];
  odom_msg.twist.covariance[8] = fix.position_covariance[8];

  // cout << std::setprecision(8) << std::fixed << endl;  // set output to fixed floating point, 8 decimal precision
  // cout << "stamp " << odom_msg.header.time << endl;
  // cout << "frame_id " << odom_msg.header.frame_id << endl;
  // cout << "x "  << odom_msg.pose.pose.position.x << endl;
  // cout << "y "  << odom_msg.pose.pose.position.y << endl;
  // cout << "z "  << odom_msg.pose.pose.position.z << endl;
  // cout << "position_covariance "  << odom_msg.pose.covariance[0] << "," << odom_msg.pose.covariance[4] << "," << odom_msg.pose.covariance[8] << endl;
  // cout << "..................... " << endl;

  return odom_msg;

  // Set default covariances
  // odom_msg.pose.covariance[6 * 0 + 0] = g_covariance_position_x;
  // odom_msg.pose.covariance[6 * 1 + 1] = g_covariance_position_y;
  // odom_msg.pose.covariance[6 * 2 + 2] = g_covariance_position_z;
  // odom_msg.pose.covariance[6 * 3 + 3] = g_covariance_orientation_x;
  // odom_msg.pose.covariance[6 * 4 + 4] = g_covariance_orientation_y;
  // odom_msg.pose.covariance[6 * 5 + 5] = g_covariance_orientation_z;

  // // Take covariances from GPS
  // if (g_trust_gps) {
  //   if (fix.position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN
  //       || fix.position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {
  //     // Fill in completely
  //     for (int i = 0; i <= 2; i++) {
  //       for (int j = 0; j <= 2; j++) {
  //         odom_msg.pose.covariance[6 * i + j] = msg->position_covariance[3 * i + j];
  //       }
  //     }
  //   } else if (msg->position_covariance_type
  //       == sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
  //     // Only fill in diagonal
  //     for (int i = 0; i <= 2; i++) {
  //       pose_msg->pose.covariance[6 * i + i] = msg->position_covariance[3 * i + i];
  //     }
  //   }
  // }


}


Odometry gps_to_pose_conversion(GPSFix fix)
{

  g_got_imu = true;

  // Specify whether covariances should be set manually or from GPS
  g_trust_gps = true;

  // Get manual parameters
  g_covariance_position_x = 4.0;
  g_covariance_position_y = 4.0;
  g_covariance_position_z = 100.0;
  g_covariance_orientation_x = 0.02;
  g_covariance_orientation_y = 0.02;
  g_covariance_orientation_z = 0.11;
  g_frame_id = "world";
  g_tf_child_frame_id = "base_link";

  // Specify whether to publish pose or not
  g_publish_pose = true;

  // Set gps_init parameters
  g_lat_ref = 0.0;
  g_lon_ref = 0.0;
  g_alt_ref = 0.0;
  g_its = 1;  // default number of fixes
  g_mode = MODE_WAIT; // MODE_AVERAGE;  // average by default
  
  // sleep 100 ms
  // eCAL::Process::SleepMS(200);

  if (!gps_ref_is_init){
    set_gps_reference(fix);
  }
  else if (gps_ref_is_init){
    odom_msg = gps_callback(fix);
  }

  
  return odom_msg;
}
