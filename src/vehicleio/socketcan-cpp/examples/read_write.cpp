#include "socketcan_cpp/socketcan_cpp.h"
#include <string>
#include <iostream>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>
#include <unistd.h>

// IPC
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

using namespace std;

// // ------------------------- MAIN CONTROL VARIABLE DEFAULT VALUES ------------------
// int steeringAngle = 0; // -30 to 30
// int throttle = 0;      // 0 to 255
// int brake = 0;         // 0 to 255
// int parkTarget = 1;    // 1 = HandBrake ON, 0 = HandBrake OFF
// int gearTarget = 3;    // 4 = Drive, 3 = Neutral, 2 = Reverse

// // Used only while manual control
// int speedLimit = 5;   // Caps the throttle value
// bool useGears = true; // Used to drive safely with controller
// // ---------------------------------------------------------------------------------

// struct CAN_MSG
// {
//     int id;
//     int data[8] = {0};
// };

// // LIMITS
// // Steering Angle Resolution = 0.06 deg
// const int steerMaxAngle = 30;
// const int steerMinAngle = -30;
// const int steerMaxInput1 = 3;
// const int steerMaxInput2 = 235; // modifed this value, original 234
// const int steerMinInput1 = 0;
// const int steerMinInput2 = 0;

// // Throttle
// const int driveMaxInput1 = 255;
// const int driveMaxInput2 = 0;
// const int driveMinInput1 = 0;
// const int driveMinInput2 = 0;
// const int gearShiftIncrement = 5;

// // Brake
// const int brakeLimit = 100;
// const int brakeMaxInput1 = 100;
// const int brakeMaxInput2 = 0;
// const int brakeMinInput1 = 0;
// const int brakeMinInput2 = 0;

// // // CALCULATE ACTUAL MIN MAX VALUES
// const int steerMaxVal = (steerMaxInput1 * 255) + steerMaxInput2;
// const int steerMinVal = (steerMinInput1 * 255) + steerMinInput2;
// const int driveMaxVal = (driveMaxInput1 * 255) + driveMaxInput2;
// const int driveMinVal = (driveMinInput1 * 255) + driveMinInput2;
// const int brakeMaxVal = (brakeMaxInput1 * 255) + brakeMaxInput2;
// const int brakeMinVal = (brakeMinInput1 * 255) + brakeMinInput2;

// // CAN IDs (Transmit)
// const int t_vehicleModeCanID = 0x105;
// const int t_parkCanID = 0x104;
// const int t_gearCanID = 0x103;
// const int t_steerCanID = 0x102;
// const int t_brakeCanID = 0x101;
// const int t_throttleCanID = 0x100;

// // // CAN IDs (Receive)
// // const int r_bmsCanID = 0x512;
// // const int r_wheelSpeedCanID = 0x506;
// // const int r_vcuCanID = 0x505;
// // const int r_parkCanID = 0x504;
// // const int r_gearCanID = 0x503;
// // const int r_steerCanID = 0x502;
// // const int r_brakeCanID = 0x501;
// // const int r_throttleCanID = 0x500;

// // # VEHICLE MODE COMMAND
// const int headlightCtrl = 7;
// const int vehicleVINReq = 1;
// const int driveModeCtrl = 1;
// const int steerModeCtrl = 0;
// const int vehicleModeCheckSum = 0;

// // # PARK COMMAND
// const int parkEnCtrl = 1;
// const int parkCheckSum = 0;

// // # GEAR COMMAND
// const int gearEnCtrl = 1;
// // const int gearTarget = 3;
// const int gearCheckSum = 0;

// // # STEERING COMMAND
// const int steerEnCtrl = 1;
// const int steerAngleSpeed = 250;
// const int steerAngleTarget = 0;
// const int steerCheckSum = 0;

// // # BRAKE COMMAND
// const int brakeEnCtrl = 1;
// const int brakeDec = 250;
// const int brakePedalTarget = 0;
// const int brakeCheckSum = 0;

// // # THROTTLE COMMAND
// const int driveEnCtrl = 1;
// const int driveAcc = 1;
// const int driveThrottlePedalTarget = 0;
// const int driveSpeedTarget = 10;
// const int driveCheckSum = 0;

// // # FLAGS & INITIALISING VARIABLES
// // const int gearShift = False;
// // const int throttle = 0;
// // const int brake = 0;

// double map(int rec_val, int signal_min, int signal_max, int adc_min, int adc_max)
// {

//     int left_pulse = signal_max - signal_min;
//     int right_pulse = adc_max - adc_min;

//     // Convert the left range into a 0-1 range (float)

//     float valueScaled = (rec_val - signal_min) / left_pulse;

//     return adc_min + (valueScaled * right_pulse);
// }


// // send data to pix
// void send(int id, int data)
// {
//     scpp::CanFrame cf_to_write;

//     cf_to_write.id = id;
//     cf_to_write.len = 8;
// }

// int main()
// {

//     CAN_MSG Steering_Command;
//     CAN_MSG Brake_Command;
//     CAN_MSG Throttle_Command;
//     CAN_MSG Gear_Command;
//     CAN_MSG vehicleMode_Command;
//     CAN_MSG Park_Command;

//     // manual input
//     /*
//     steering angle = 30 degree
//     throttle value = 0
//     brake value = 0
//     */
//     int steering_angle = -10;
//     int throttle_value = 0;
//     int brake_value = 0;

//     // creating can id
//     Steering_Command.id = 258;

//     scpp::SocketCan socket_can;
//     if (socket_can.open("can0") == scpp::STATUS_OK)
//     {
//         while (1)
//         {
//             sleep(0.02);
//             vehicleMode_Command.data[0] = steerModeCtrl;
//             vehicleMode_Command.data[1] = driveModeCtrl;
//             vehicleMode_Command.data[2] = headlightCtrl;
//             vehicleMode_Command.data[3] = vehicleVINReq;
//             vehicleMode_Command.data[4] = 0;
//             vehicleMode_Command.data[5] = 0;
//             vehicleMode_Command.data[6] = 0;
//             vehicleMode_Command.data[7] = vehicleModeCheckSum;

//             Park_Command.data[0] = parkEnCtrl;
//             Park_Command.data[1] = parkTarget;
//             Park_Command.data[2] = 0;
//             Park_Command.data[3] = 0;
//             Park_Command.data[4] = 0;
//             Park_Command.data[5] = 0;
//             Park_Command.data[6] = 0;
//             Park_Command.data[7] = parkCheckSum;

//             Gear_Command.data[0] = gearEnCtrl;
//             Gear_Command.data[1] = gearTarget;
//             Gear_Command.data[2] = 0;
//             Gear_Command.data[3] = 0;
//             Gear_Command.data[4] = 0;
//             Gear_Command.data[5] = 0;
//             Gear_Command.data[6] = 0;
//             Gear_Command.data[7] = gearCheckSum;

//             // steering angle sorted
//             Steering_Command.data[0] = steerEnCtrl;
//             Steering_Command.data[1] = steerAngleSpeed;
//             Steering_Command.data[2] = 0;
//             // Steering_Command.data[3] = 2;
//             // Steering_Command.data[4] = 100;
//             double steering_send_initial = map(steering_angle, steerMinAngle, steerMaxAngle, steerMinVal, steerMaxVal);
//             Steering_Command.data[3] = steering_send_initial / 255;
//             Steering_Command.data[4] = ((steering_send_initial / 255) - int((steering_send_initial / 255))) * 255;
//             Steering_Command.data[5] = 0;
//             Steering_Command.data[6] = 0;
//             Steering_Command.data[7] = steerCheckSum;

//             // throttle sorted
//             Throttle_Command.data[0] = driveEnCtrl;
//             Throttle_Command.data[1] = driveAcc;
//             Throttle_Command.data[2] = 0;
//             Throttle_Command.data[3] = driveThrottlePedalTarget;
//             Throttle_Command.data[4] = 0;
//             Throttle_Command.data[5] = throttle_value;
//             Throttle_Command.data[6] = 0;
//             Throttle_Command.data[7] = driveCheckSum;

//             // brake sorted
//             Brake_Command.data[0] = brakeEnCtrl;
//             Brake_Command.data[1] = brakeDec;
//             Brake_Command.data[2] = 0;
//             Brake_Command.data[3] = brake_value;
//             Brake_Command.data[4] = 0;
//             Brake_Command.data[5] = 0;
//             Brake_Command.data[6] = 0;
//             Brake_Command.data[7] = brakeCheckSum;
//             scpp::CanFrame cf_to_write;

//             cf_to_write.id = 258;
//             cf_to_write.len = 8;
//             for (int i = 0; i < 8; ++i)
//                 cf_to_write.data[i] = Steering_Command.data[i];

//             auto write_sc_status = socket_can.write(cf_to_write);

//             if (write_sc_status != scpp::STATUS_OK)
//                 printf("something went wrong on socket write, error code : %d \n", int32_t(write_sc_status));
//             else
//                 printf("Message was written to the socket \n");
//         }
//     }
//     else
//     {
//         printf("Cannot open can socket!");
//     }
//     return 0;
// }

int main(int argc, char *argv[])

{
    eCAL::Initialize(argc, argv, "read_write");
   //   publisher topic name -> pix_feedback
    eCAL::CPublisher publisher{"ctrl"};
    // eCAL::CSubscriber subscriber{"ctrl"};
    // subscriber.AddReceiveCallback(std::bind(&HelloWorldCallback,std::placeholders::_1, std::placeholders::_2));
    // Just don't exit
    while (eCAL::Ok())
    {
        std::string temp = "Hello World";
        publisher.Send(temp.c_str(),sizeof(temp));
        std::cout<<temp<<" send successfully "<<std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // finalize eCAL API
    eCAL::Finalize();

}