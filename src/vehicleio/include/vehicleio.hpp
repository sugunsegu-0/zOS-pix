#pragma once

#ifndef VEHICLE_IO_HPP
#define VEHICLE_IO_HPP

#include "cpp_headers.hpp"
#include "pix_data.hpp"
#include "control/control-DS.hpp"
#include "vehicleio/vehicle_DS.hpp"

// IPC
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

#include "socketcan_cpp/socketcan_cpp.h"

class vehicleIO
{
    public:
        vehicleIO();
        FEEDBACK feed_from_pix_control();
        float input_parse(float rec_val,float signal_min,float signal_max,float adc_min,float adc_max);
        float absolute(float val,float least_count);
        void send_to_pix();
        void receive_data(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_);
        ctrl parse_data(char* buffer);
        void debug();
        void default_values_to_pix();
        bool write_steering_angle(float steering_angle);
        bool write_throttle(float throttle_value);
        bool write_gear(float gearTarget);
        bool write_park(float parkTarget);
        bool write_vcu();
        std::unique_ptr<eCAL::CSubscriber> subscriber;
        std::unique_ptr<eCAL::CPublisher> publisher;
        void send_to_control(FEEDBACK feed);
        float check(float val,float MAX_VAL);

    private:
        
        CAN_MSG Steering_Command;
        CAN_MSG Throttle_Command;
        CAN_MSG Park_Command;
        CAN_MSG Gear_Command;
        CAN_MSG Vehicle_Command;
        // ------------------------- MAIN CONTROL VARIABLE DEFAULT VALUES ------------------
        float steering_angle = 0; // -30 to 30
        float throttle_value = 0;      // 0 to 255
        int brake = 0;         // 0 to 255
        int parkTarget = 0;    // 1 = HandBrake ON, 0 = HandBrake OFF
        int gearTarget = 4;    // 4 = Drive, 3 = Neutral, 2 = Reverse
        bool init_flag = false;
        // Used only while manual control
        int speedLimit = 5;   // Caps the throttle value
        bool useGears = true; // Used to drive safely with controller
        // ------------------------- MAIN CONTROL VARIABLE DEFAULT VALUES ------------------
        scpp::SocketCan socket_can_write;
        scpp::SocketCan socket_can_read;
        std::shared_ptr<ctrl> frame;
        double iterator=0;

};

#endif