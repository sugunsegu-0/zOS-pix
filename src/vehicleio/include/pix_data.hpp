#pragma once
#ifndef PIX_DATA_HPP
#define PIX_DATA_HPP

#include <stdlib.h>

/*---------------------------CAN ID receive--------------------*/
const int r_bmsCanID = 512;
const int r_wheelSpeedCanID = 506;
const int r_vcuCanID = 1285;
const int r_parkCanID = 1284;
const int r_gearCanID = 1283;
const int r_steerCanID = 1282;
const int r_brakeCanID = 1281;
const int r_throttleCanID = 1280;
/*---------------------------CAN ID receive--------------------*/


/*---------------------------CAN ID transmit--------------------*/
const int t_vehicleModeCanID = 261;
const int t_parkCanID = 260;
const int t_gearCanID = 259;
const int t_steerCanID = 258;
const int t_brakeCanID = 257;
const int t_throttleCanID = 256;
/*---------------------------CAN ID transmit--------------------*/


/*---------------------------Steering values--------------------*/
const int steerMaxAngle = 30;
const int steerMinAngle = -30;
const int steerMaxInput1 = 3;
const int steerMaxInput2 = 232;
const int steerMinInput1 = 0;
const int steerMinInput2 = 0;
/*---------------------------Steering values--------------------*/


/*---------------------------Speed values--------------------*/
const int speedMax = 40;
const int speedMin = 0;
const int speedMaxInput1 = 43;
const int speedMaxInput2 = 184;
const int speedMinInput1 = 212;
const int speedMinInput2 = 208;
const int speedMaxLimit = 40;
/*---------------------------Speed values--------------------*/


/*---------------------------Throttle values--------------------*/
const int ThrottleMaxVal = 70;
const int driveMaxInput1 = 255;
const int driveMaxInput2 = 0;
const int driveMinInput1 = 0;
const int driveMinInput2 = 0;
const int gearShiftIncrement = 5;
/*---------------------------Throttle values--------------------*/


/*---------------------------Brake values--------------------*/
const int brakeLimit = 100;
const int brakeMaxInput1 = 100;
const int brakeMaxInput2 = 0;
const int brakeMinInput1 = 0;
const int brakeMinInput2 = 0;
/*---------------------------Brake values--------------------*/


/*---------------------------Commands--------------------*/
// # VCU COMMAND
const int headlightCtrl = 7;
const int vehicleVINReq = 1;
const int driveModeCtrl = 1;
const int steerModeCtrl = 0;
const int vehicleModeCheckSum = 0;

// # PARK COMMAND
const int parkEnCtrl = 1;
const int parkCheckSum = 0;

// # GEAR COMMAND
const int gearEnCtrl = 1;
const int gearCheckSum = 0;

// # STEERING COMMAND
const int steerEnCtrl = 1;
const int steerAngleSpeed = 250;
const int steerAngleTarget = 0;
const int steerCheckSum = 0;

// # BRAKE COMMAND
const int brakeEnCtrl = 1;
const int brakeDec = 250;
const int brakePedalTarget = 0;
const int brakeCheckSum = 0;

// # THROTTLE COMMAND
const int driveEnCtrl = 1;
const int driveAcc = 1;
const int driveThrottlePedalTarget = 0;
const int driveSpeedTarget = 10;
const int driveCheckSum = 0;
/*---------------------------Commands--------------------*/


/*---------------------------Limits--------------------*/
const float steerMaxVal = 997;
const float speedMaxVal = (speedMaxInput1*255) + speedMaxInput2;
const float speedMinVal = (speedMinInput1*255) + speedMinInput2;
const float brakeMaxVal = (brakeMaxInput1 * 255) + brakeMaxInput2;
const float least_count_steering = 0.06;
const float least_count_throttle = 0.61;
/*---------------------------Limits--------------------*/


struct CAN_MSG
{
int id;
int data[8]={0x00};

};


#endif