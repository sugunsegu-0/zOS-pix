#include "socketcan_cpp/socketcan_cpp.h"
#include <string>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>
#include <unistd.h>
#include <thread>

// IPC
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

//commons
#include "commons.hpp"

//control DS
#include "control/control-DS.hpp"
#include "vehicleio/vehicle_DS.hpp"

using namespace std;

// struct FEEDBACK
// {
//     float steering_angle;
//     float vehicle_speed;
//     int gear_fit; 
//     int gear_actual;
//     int park_fit;
//     int park_actual;
//     int bat_led_data_acid;
//     int bat_current;
//     int bat_voltage;
//     int bat_soc;
//     int wheel_speed_fl;
//     int wheel_speed_fr;
//     int wheel_speed_rl;
//     int wheel_speed_rr;
// };
FEEDBACK feedback;

struct CAN_MSG
{
    int id;
    int data[8] = {0};
};

// ------------------------- MAIN CONTROL VARIABLE DEFAULT VALUES ------------------
int steeringAngle = 0; // -30 to 30
int throttle = 0;      // 0 to 255
int brake = 0;         // 0 to 255
int parkTarget = 1;    // 1 = HandBrake ON, 0 = HandBrake OFF
int gearTarget = 3;    // 4 = Drive, 3 = Neutral, 2 = Reverse

// Used only while manual control
int speedLimit = 5;   // Caps the throttle value
bool useGears = true; // Used to drive safely with controller
// ---------------------------------------------------------------------------------

/*---------------------------CAN ID--------------------*/
const int r_bmsCanID = 512;
const int r_wheelSpeedCanID = 506;
const int r_vcuCanID = 505;
const int r_parkCanID = 504;
const int r_gearCanID = 503;
const int r_steerCanID = 1280;
const int r_brakeCanID = 501;
const int r_throttleCanID = 500;

const int id_arr[8] = {r_throttleCanID,r_brakeCanID,r_steerCanID,r_gearCanID,r_parkCanID,r_vcuCanID,r_wheelSpeedCanID,r_bmsCanID};

const int steerMaxAngle = 30;
const int steerMinAngle = -30;
const int steerMaxInput1 = 3;
const int steerMaxInput2 = 232;
const int speedMax = 45;
const int speedMin = 0;
const int speedMaxInput1 = 45;
const int speedMaxInput2 = 254;


// Throttle
const int driveMaxInput1 = 255;
const int driveMaxInput2 = 0;
const int driveMinInput1 = 0;
const int driveMinInput2 = 0;
const int gearShiftIncrement = 5;

// Brake
const int brakeLimit = 100;
const int brakeMaxInput1 = 100;
const int brakeMaxInput2 = 0;
const int brakeMinInput1 = 0;
const int brakeMinInput2 = 0;


// CAN IDs (Transmit)
const int t_vehicleModeCanID = 0x105;
const int t_parkCanID = 0x104;
const int t_gearCanID = 0x103;
const int t_steerCanID = 0x102;
const int t_brakeCanID = 0x101;
const int t_throttleCanID = 0x100;


// # VEHICLE MODE COMMAND
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
// const int gearTarget = 3;
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

int steerMaxVal = (steerMaxInput1*255) + steerMaxInput2;
int speedMaxVal = (speedMaxInput1*255) + speedMaxInput2;
int brakeMaxVal = (brakeMaxInput1 * 255) + brakeMaxInput2;

int input_parse(int rec_val,int signal_min,int signal_max,int adc_min,int adc_max)
{
    
    int left_pulse = signal_max - signal_min;
    int right_pulse = adc_max - adc_min;

    // Convert the left range into a 0-1 range (float)

    float valueScaled = (rec_val - signal_min) / (float)left_pulse;

    return adc_min + (valueScaled * right_pulse);
}

FEEDBACK feed_from_pix_to_control()
{
    FEEDBACK feedback;
    scpp::SocketCan socket_can;
    scpp::CanFrame fr;
    if (socket_can.open("vcan0") == scpp::STATUS_OK)
    {
    // std::cout<<"fr.id "<<fr.id<<std::endl;
    while(socket_can.read(fr) == scpp::STATUS_OK)
    {
        if(fr.id == r_steerCanID){
            int data = fr.data[3]*255 + fr.data[4];
            int steering_angle = input_parse(data,0,steerMaxVal,-30,30);
            std::cout<<"steering angle "<<steering_angle<<std::endl;
            feedback.steering_angle = steering_angle;
        }
        if(fr.id == r_vcuCanID)
        {
            int data = fr.data[2]*255 + fr.data[3];
            int vehicle_speed = input_parse(data,0,speedMaxVal,speedMin,speedMax);
            std::cout<<vehicle_speed<<std::endl;
            feedback.vehicle_speed = vehicle_speed;
        }
        if(fr.id == r_gearCanID)
        {
            int gear_fit = fr.data[1];
            int gear_actual = fr.data[0];
            std::cout<<gear_fit<<" "<<gear_actual<<std::endl;
            feedback.gear_actual = gear_actual;
            feedback.gear_fit = gear_fit;
        }
        if(fr.id == r_parkCanID)
        {
            //park fit data
            int park_fit = fr.data[1];
            int park_actual = fr.data[0];
            std::cout<<park_fit<<" "<<park_actual<<std::endl;
            feedback.park_actual = park_actual;
            feedback.park_fit = park_fit;
        }
        if(fr.id == r_bmsCanID)
        {
            int bat_led_data_acid = fr.data[6]*255 + fr.data[7];      
            int bat_current= fr.data[2]*255 + fr.data[3];
            int bat_voltage = fr.data[0]*255 + fr.data[1];
            int bat_soc = fr.data[4]*255 + fr.data[5];
            std::cout<<bat_led_data_acid<<" "<<bat_current<<" "<<bat_voltage<<" "<<bat_soc<<std::endl;
            feedback.bat_led_data_acid = bat_led_data_acid;
            feedback.bat_current = bat_current;
            feedback.bat_voltage = bat_voltage;
            feedback.bat_soc = bat_soc;
        }
        if(fr.id == r_wheelSpeedCanID)
        {
            int wheel_speed_fl_input = fr.data[0]*255 + fr.data[1];
            int wheel_speed_fl = input_parse(wheel_speed_fl_input,0,(speedMaxVal-1),speedMin,speedMax);
            feedback.wheel_speed_fl = wheel_speed_fl;

            int wheel_speed_fr_input = fr.data[2]*255 + fr.data[3];
            int wheel_speed_fr = input_parse(wheel_speed_fr_input,0,(speedMaxVal-1),speedMin,speedMax);
            feedback.wheel_speed_fr = wheel_speed_fr;

            int wheel_speed_rl_input = fr.data[4]*255 + fr.data[5];
            int wheel_speed_rl = input_parse(wheel_speed_rl_input,0,(speedMaxVal-1),speedMin,speedMax);
            feedback.wheel_speed_rl = wheel_speed_rl;

            int wheel_speed_rr_input = fr.data[6]*255 + fr.data[7];
            int wheel_speed_rr = input_parse(wheel_speed_rr_input,speedMin,speedMax,0,(speedMaxVal-1));
            feedback.wheel_speed_rr = wheel_speed_rr;
        }
        
        /*
        TODO:
        -> Create a structure to store all the feedback ✔️
        */
    }
    }
    return feedback;
    }

void send_to_pix(ctrl controls_cmd)
{
    sleep(0.02);
    CAN_MSG Steering_Command;
    CAN_MSG Brake_Command;
    CAN_MSG Throttle_Command;
    CAN_MSG Gear_Command;
    CAN_MSG vehicleMode_Command;
    CAN_MSG Park_Command;

    // manual input
    /*
    steering angle = 30 degree
    throttle value = 0
    brake value = 0
    */
    int steering_angle = controls_cmd.steer;
    int throttle_value = controls_cmd.linear_v;
    int brake_value = 0;

    scpp::SocketCan socket_can;
    if (socket_can.open("vcan0") == scpp::STATUS_OK)
    {
            vehicleMode_Command.data[0] = steerModeCtrl;
            vehicleMode_Command.data[1] = driveModeCtrl;
            vehicleMode_Command.data[2] = headlightCtrl;
            vehicleMode_Command.data[3] = vehicleVINReq;
            vehicleMode_Command.data[4] = 0;
            vehicleMode_Command.data[5] = 0;
            vehicleMode_Command.data[6] = 0;
            vehicleMode_Command.data[7] = vehicleModeCheckSum;

            Park_Command.data[0] = parkEnCtrl;
            Park_Command.data[1] = parkTarget;
            Park_Command.data[2] = 0;
            Park_Command.data[3] = 0;
            Park_Command.data[4] = 0;
            Park_Command.data[5] = 0;
            Park_Command.data[6] = 0;
            Park_Command.data[7] = parkCheckSum;

            Gear_Command.data[0] = gearEnCtrl;
            Gear_Command.data[1] = gearTarget;
            Gear_Command.data[2] = 0;
            Gear_Command.data[3] = 0;
            Gear_Command.data[4] = 0;
            Gear_Command.data[5] = 0;
            Gear_Command.data[6] = 0;
            Gear_Command.data[7] = gearCheckSum;

            // steering angle sorted
            Steering_Command.data[0] = steerEnCtrl;
            Steering_Command.data[1] = steerAngleSpeed;
            Steering_Command.data[2] = 0;
            // Steering_Command.data[3] = 2;
            // Steering_Command.data[4] = 100;
            int steering_send_initial = input_parse(steering_angle,-30,30,0,steerMaxVal);
            Steering_Command.data[3] = steering_send_initial / 255;
            Steering_Command.data[4] = ((steering_send_initial / 255) - int((steering_send_initial / 255))) * 255;
            Steering_Command.data[5] = 0;
            Steering_Command.data[6] = 0;
            Steering_Command.data[7] = steerCheckSum;

            // throttle sorted
            Throttle_Command.data[0] = driveEnCtrl;
            Throttle_Command.data[1] = driveAcc;
            Throttle_Command.data[2] = 0;
            Throttle_Command.data[3] = driveThrottlePedalTarget;
            Throttle_Command.data[4] = 0;
            Throttle_Command.data[5] = throttle_value;
            Throttle_Command.data[6] = 0;
            Throttle_Command.data[7] = driveCheckSum;

            // brake sorted
            Brake_Command.data[0] = brakeEnCtrl;
            Brake_Command.data[1] = brakeDec;
            Brake_Command.data[2] = 0;
            Brake_Command.data[3] = brake_value;
            Brake_Command.data[4] = 0;
            Brake_Command.data[5] = 0;
            Brake_Command.data[6] = 0;
            Brake_Command.data[7] = brakeCheckSum;
            scpp::CanFrame cf_to_write;

            cf_to_write.id = 258;
            cf_to_write.len = 8;
            for (int i = 0; i < 8; ++i)
                cf_to_write.data[i] = Steering_Command.data[i];

            auto write_sc_status = socket_can.write(cf_to_write);

            if (write_sc_status != scpp::STATUS_OK)
                printf("something went wrong on socket write, error code : %d \n", int32_t(write_sc_status));
            else
                printf("Message was written to the socket \n");
    }
}

/*

Subscriber to controller -> infinite loop [ --> data aaya --> data send to car, feedback aaya, feedback publish top controller]
*/

/*
TODO:-
-> Subscribe to controller ✔️
-> Publisher to controller ✔️
-> Put reading code inside the receive_data  ✔️
-> Include commons and opencv in cmake ✔️
-> receive data structure from control ✔️
-> include boost in the cmake ✔️
-> structure added to send_to_pix ✔️
-> sab kuch chal raha hai ✔️

*/


void receive_data(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
{
//   std::cout << "Received Message: " << message << std::endl;
    std::cout<<"Inside Hello world callback"<<std::endl;

    // feedback structure
    FEEDBACK feedb;
    // // data from control 
    char* buffer=static_cast<char*>(data_->buf);
    std::stringstream ss(buffer);
    Serialize<ctrl> IPC;
    ctrl data;
    data = IPC.deserialize(ss,data);
    // sending the data received from controller to car
    send_to_pix(data);
}

int main(int argc, char *argv[])
{
    eCAL::Initialize(argc, argv, "read_parse");
    // publisher topic name -> pix_feedback
    eCAL::CPublisher publisher("pix_feedback");
    eCAL::CSubscriber subscriber{"ctrl"};
    // std::make_unique<eCAL::CPublisher> publisher{"path"};
    // unique_ptr<eCAL::CPublisher> publisher = std::make_unique<eCAL::CPublisher>("control_feedback");
    subscriber.AddReceiveCallback(std::bind(&receive_data,std::placeholders::_1,std::placeholders::_2));
    // Just don't exit
  
    while (eCAL::Ok())
    {   
        FEEDBACK feed;
        feed = feed_from_pix_to_control();
        Serialize<FEEDBACK> IPC_send;
        std::stringstream ss;
        IPC_send.serialize(feed,ss);
        std::string temp = ss.str();
        publisher.Send(temp.c_str(),sizeof(temp));
        cout<<"\nsent from publisher"<<endl;
    }

    // finalize eCAL API
    eCAL::Finalize();

}