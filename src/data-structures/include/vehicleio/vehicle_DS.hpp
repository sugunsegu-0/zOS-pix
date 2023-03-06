#pragma once
#ifndef VEHICHLE_DS_HPP
#define VEHICLE_DS_HPP

#include <iostream>
#include <string>
#include <sstream>


struct FEEDBACK
{
    float steering_angle;
    float vehicle_speed;
    int gear_fit; 
    int gear_actual;
    int park_fit;
    int park_actual;
    int bat_led_data_acid;
    int bat_current;
    int bat_voltage;
    int bat_soc;
    int wheel_speed_fl;
    int wheel_speed_fr;
    int wheel_speed_rl;
    int wheel_speed_rr;
};


namespace boost {
namespace serialization {

    template <typename Ar>
    void serialize(Ar &ar, FEEDBACK &f, unsigned)
    {
    ar &f.steering_angle;
    ar &f.vehicle_speed;
    ar &f.gear_fit; 
    ar &f.gear_actual;
    ar &f.park_fit;
    ar &f.park_actual;
    ar &f.bat_led_data_acid;
    ar &f.bat_current;
    ar &f.bat_voltage;
    ar &f.bat_soc;
    ar &f.wheel_speed_fl;
    ar &f.wheel_speed_fr;
    ar &f.wheel_speed_rl;
    ar &f.wheel_speed_rr;
    }
}
}
#endif