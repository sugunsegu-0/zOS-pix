#pragma once
#ifndef GPS_STRUCTURES_HPP
#define GPS_STRUCTURES_HPP

#include<iostream>

#include "imu_data.hpp"

struct GPSStatus{
// int16_t STATUS_NO_FIX=-1;
// int16_t STATUS_FIX=0;
int16_t STATUS_SBAS_FIX=1;
int16_t STATUS_GBAS_FIX=2;
// int16_t STATUS_DGPS_FIX=18;
int16_t STATUS_WAAS_FIX=33;
uint16_t SOURCE_NONE=0;
uint16_t SOURCE_GPS=1;
uint16_t SOURCE_POINTS=2;
uint16_t SOURCE_DOPPLER=4;
uint16_t SOURCE_ALTIMETER=8;
uint16_t SOURCE_MAGNETIC=16;
uint16_t SOURCE_GYRO=32;
uint16_t SOURCE_ACCEL=64;
Header header;
uint16_t satellites_used=0;
// int32_t satellite_used_prn[100];
uint16_t satellites_visible=0;
// int32_t satellite_visible_prn[100];
// int32_t satellite_visible_z[100];
// int32_t satellite_visible_azimuth[100];
// int32_t satellite_visible_snr[100];
int16_t status=0;
uint16_t motion_source=0;
uint16_t orientation_source=0;
uint16_t position_source=0;
};

struct GPSFix{
	uint8_t COVARIANCE_TYPE_UNKNOWN=0;
	uint8_t COVARIANCE_TYPE_APPROXIMATED=1;
	uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN=2;
	uint8_t COVARIANCE_TYPE_KNOWN=3;
	Header header;
	GPSStatus status;
	double latitude=0.0;
	double longitude=0.0;
	double altitude=0.0;
	double track=0.0;
	double speed=0.0;
	double climb=0.0;
	double pitch=0.0;
	double roll=0.0;
	double dip=0.0;
	double time=0.0;
	double gdop=0.0;
	double pdop=0.0;
	double hdop=0.0;
	double vdop=0.0;
	double tdop=0.0;
	double err=0.0;
	double err_horz=0.0;
	double err_vert=0.0;
	double err_track=0.0;
	double err_speed=0.0;
	double err_climb=0.0;
	double err_time=0.0;
	double err_pitch=0.0;
	double err_roll=0.0;
	double err_dip=0.0;
	double position_covariance[9];
	uint8_t position_covariance_type=0;
};
namespace boost {
namespace serialization {
 template <typename Ar> void serialize(Ar& ar, GPSStatus& status, unsigned) 
    {
    	// ar & status.STATUS_NO_FIX;
    	// ar & status.STATUS_FIX;
    	ar & status.STATUS_SBAS_FIX;
    	ar & status.STATUS_GBAS_FIX;
    	// ar & status.STATUS_DGPS_FIX;
    	ar & status.STATUS_WAAS_FIX;
    	ar & status.SOURCE_NONE;
    	ar & status.SOURCE_GPS;
    	ar & status.SOURCE_POINTS;
    	ar & status.SOURCE_DOPPLER;
    	ar & status.SOURCE_ALTIMETER;
    	ar & status.SOURCE_MAGNETIC;
    	ar & status.SOURCE_GYRO;
    	ar & status.SOURCE_ACCEL;
    	ar & status.header;
    	ar & status.satellites_used;
    	ar & status.satellites_visible;
    	ar & status.status;
    	ar & status.motion_source;
    	ar & status.orientation_source;
    	ar & status.position_source;
    }

    template <typename Ar> void serialize(Ar& ar, GPSFix& fix, unsigned) 
    {
    	ar & fix.COVARIANCE_TYPE_UNKNOWN;
    	ar & fix.COVARIANCE_TYPE_APPROXIMATED;
    	ar & fix.COVARIANCE_TYPE_DIAGONAL_KNOWN;
    	ar & fix.COVARIANCE_TYPE_KNOWN;
    	ar & fix.header;
    	ar & fix.status;
    	ar & fix.latitude;
    	ar & fix.longitude;
    	ar & fix.altitude;
    	ar & fix.track;
    	ar & fix.speed;
    	ar & fix.climb;
    	ar & fix.pitch;
    	ar & fix.roll;
    	ar & fix.dip;
    	ar & fix.time;
    	ar & fix.gdop;
    	ar & fix.pdop;
    	ar & fix.hdop;
    	ar & fix.vdop;
    	ar & fix.tdop;
    	ar & fix.err;
    	ar & fix.err_horz;
    	ar & fix.err_vert;
    	ar & fix.err_track;
    	ar & fix.err_speed;
    	ar & fix.err_climb;
    	ar & fix.err_time;
    	ar & fix.err_pitch;
    	ar & fix.err_roll;
    	ar & fix.err_dip;
    	ar & fix.position_covariance;
    	ar & fix.position_covariance_type;
    }
}
}
#endif