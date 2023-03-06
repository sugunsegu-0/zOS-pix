#pragma once

#include "mscl/mscl.h"

//Example: Get Current Configuration
//  Shows how to read current configuration settings an Inertial Device.
static void getCurrentConfig(mscl::InertialNode& node)
{
    //many other settings are available than shown below
    //reference the documentation for the full list of commands

    //if the node supports AHRS/IMU
    if(node.features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU))
    {
        //get a list of the AHRS/IMU channels currently active on the Node
        mscl::MipChannels ahrsImuActiveChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU);

        std::cout << "AHRS/IMU Channels" << std::endl;
        std::cout << "-----------------" << std::endl;
        for(mscl::MipChannel ch : ahrsImuActiveChs)
        {
            std::cout << "Channel Field: " << std::hex << ch.channelField() << std::endl;
            std::cout << "Sample Rate: " << ch.sampleRate().prettyStr() << std::endl << std::endl;
        }
    }

    //if the node supports Estimation Filter
    if(node.features().supportsCategory(mscl::MipTypes::CLASS_ESTFILTER))
    {
        //get a list of the Estimation Filter channels currently active on the Node
        mscl::MipChannels estFilterActiveChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER);

        std::cout << std::endl;
        std::cout << "Estimation Filter Channels" << std::endl;
        std::cout << "--------------------------" << std::endl;
        for(mscl::MipChannel ch : estFilterActiveChs)
        {
            std::cout << "Channel Field: " << std::hex << ch.channelField() << std::endl;
            std::cout << "Sample Rate: " << ch.sampleRate().prettyStr() << std::endl << std::endl;
        }
    }

    //if the node supports GNSS
    if(node.features().supportsCategory(mscl::MipTypes::CLASS_GNSS))
    {
        //get a list of the GNSS channels currently active on the Node
        mscl::MipChannels gnssActiveChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_GNSS);

        std::cout << std::endl;
        std::cout << "GNSS Channels" << std::endl;
        std::cout << "-------------" << std::endl;
        for(mscl::MipChannel ch : gnssActiveChs)
        {
            std::cout << "Channel Field: " << std::hex << ch.channelField() << std::endl;
            std::cout << "Sample Rate: " << ch.sampleRate().prettyStr() << std::endl << std::endl;
        }
    }

    // std::cout << "Altitude Aiding enabled?: " << node.getAltitudeAid() << std::endl;

    // mscl::PositionOffset offset = node.getAntennaOffset();
    // std::cout << "Antenna Offset: x=" << offset.x() << " y=" << offset.y() << " z=" << offset.z() << std::endl;

    // std::cout << "Pitch/Roll Aiding enabled?: " << node.getPitchRollAid() << std::endl;
}