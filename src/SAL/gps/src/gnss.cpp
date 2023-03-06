#include <ctime>
#include <iomanip>
// #include <iostream>
#include <sstream>
// #include <thread>
#include <chrono>
#include <math.h>
#include <signal.h>

// #include "commons.hpp"

//Include GPSD - standard linux GNSS daemon.
// #include <gps.h>
// #include <libgpsmm.h>


#include "../include/gpsd_config.h"  /* must be before all includes */
#include <ctype.h>
#include <curses.h>
#include <errno.h>
#ifdef HAVE_GETOPT_LONG
       #include <getopt.h>
#endif
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../include/gps.h"
#include "../include/gps_json.h"   /* for GPS_JSON_RESPONSE_MAX */
#include "../include/compiler.h"   /* for UNUSED */
#include "../include/gpsdclient.h"
#include "../include/os_compat.h"
#include "../include/timespec.h"



//Include message structures.
// #include "localization/localizationDS.hpp"

//Include eCal package for communication.
// #include <ecal/ecal.h>
// #include <ecal/msg/string/publisher.h>

//For Serialization.
#include <commons.hpp>

#include "gps_to_pose_conversion_node.hpp"

//Structures declaration.
Header header;
GPSStatus gpsStatus;
GPSFix gpsFix;

using namespace std;

void signal_callback_handler(int signum) {

    if (path_file.is_open())
    {
      for (auto it = path.begin(); it != path.end(); ++it)
        path_file << *it << "\n";
        

      path_file.close();
    }
    
   // Terminate program
   exit(signum);
}

static struct gps_data_t gpsdata;
static struct fixsource_t source;

/* This gets called once for each new GPS sentence. */
GPSFix update_gps_panel(struct gps_data_t *gpsd_data)
{
    /* Fill in the latitude. */
    if (gpsd_data->fix.mode >= MODE_2D) {

        auto now = std::chrono::high_resolution_clock::now();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        
        header.frame_id = "gps_receiver";
        header.time = nanoseconds/1e9;

        double heading = gpsd_data->fix.NED.relPosH;
        double length = gpsd_data->fix.NED.relPosL;
        
        gpsFix.header = header;
        gpsFix.status.status = gpsd_data->fix.mode;
        const auto service = gpsd_data->fix.mode;
        gpsFix.latitude = isnan(gpsd_data->fix.latitude) ? 0.0 : gpsd_data->fix.latitude;
        gpsFix.longitude = isnan(gpsd_data->fix.longitude) ? 0.0 : gpsd_data->fix.longitude;
        gpsFix.altitude = isnan(gpsd_data->fix.altHAE) ? 0.0 : gpsd_data->fix.altHAE;

        double hEst = isnan(gpsd_data->fix.eph) ? 0.0 : gpsd_data->fix.eph;
        double vEst = isnan(gpsd_data->fix.epv) ? 0.0 : gpsd_data->fix.epv;

        gpsFix.position_covariance[0] = std::pow(hEst, 2);
        gpsFix.position_covariance[4] = std::pow(hEst, 2);
        gpsFix.position_covariance[8] = std::pow(vEst, 2);
        gpsFix.position_covariance_type = gpsFix.COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gpsFix.track = gpsd_data->fix.track;
        gpsFix.speed = gpsd_data->fix.speed;
        gpsFix.climb = gpsd_data->fix.climb;
        gpsFix.pitch = gpsd_data->attitude.pitch;
        gpsFix.roll = gpsd_data->attitude.roll;
        gpsFix.dip = gpsd_data->attitude.dip;
        // gpsFix.time = gpsd_data->fix.time;
        gpsFix.gdop = gpsd_data->dop.gdop;
        gpsFix.pdop = gpsd_data->dop.pdop;
        gpsFix.hdop = gpsd_data->dop.hdop;
        gpsFix.vdop = gpsd_data->dop.vdop;
        gpsFix.tdop = gpsd_data->dop.tdop;
        gpsFix.err = gpsd_data->fix.eph;
        gpsFix.err_horz = gpsd_data->fix.eph;
        gpsFix.err_vert = gpsd_data->fix.epv;
        gpsFix.err_track = gpsd_data->fix.epd;
        gpsFix.err_speed = gpsd_data->fix.eps;
        gpsFix.err_climb = gpsd_data->fix.epc;
        gpsFix.err_time = gpsd_data->fix.ept;

        // cout << std::setprecision(8) << std::fixed << endl;  // set output to fixed floating point, 8 decimal precision
        // cout << "stamp " << gpsFix.header.time << endl;
        // cout << "frame_id " << gpsFix.header.frame_id << endl;
        // // cout << "status "  << status << endl;
        // cout << "latitude "  << gpsFix.latitude << endl;
        // cout << "longitude "  << gpsFix.longitude << endl;
        // cout << "altitude "  << gpsFix.altitude << endl;
        // cout << "heading "  << gpsd_data-> << endl;
        // cout << "speed "  << gpsd_data->fix.speed << endl;
        // cout << std::setprecision(3) << std::fixed << endl;  // set output to fixed floating point, 8 decimal precision
        // cout << "Horizontal Error" << gpsd_data->fix.eph << ", Vertical Error" << gpsd_data->fix.epv << endl;
        
        // cout << "position_covariance "  << gpsFix.position_covariance[0] << "," << gpsFix.position_covariance[4] << "," << gpsFix.position_covariance[8] << endl;
        // cout << "............ " << endl;
        
        
    } 

    return gpsFix;
}

int main(int argc, char *argv[])
{
    unsigned int flags = WATCH_ENABLE;
    int wait_clicks = 0;  /* cycles to wait before gpsd timeout */
    /* buffer to hold one JSON message */
    char message[GPS_JSON_RESPONSE_MAX];
    const char *optstring = "?D:hil:msu:V";


    //eCal Publisher.
    eCAL::Initialize(argc, argv, "GNSS Publisher");

    // Create a String Publisher that publishes on the topic "gnss"
    eCAL::string::CPublisher<std::string> gnss_publisher("gps/fix");
    eCAL::string::CPublisher<std::string> gps_odom_pub("gps/odom");

    //For serialization of msg.
    std::string ser_gnss;
    std::string ser_odom;


    // Control C catch to save Waypoints
    signal(SIGINT, signal_callback_handler);


    /* Open the stream to gpsd. */
    if (gps_open(source.server, source.port, &gpsdata) != 0) {
        cout << "cgps: no gpsd running or network error" << endl;
        exit(EXIT_FAILURE);
    }

    if (source.device != NULL)
        flags |= WATCH_DEVICE;

    gps_stream(&gpsdata, flags, source.device);

    /* heart of the client */
    for (;;) {
        int ret;
        /* wait 1/2 second for gpsd */
        ret = gps_waiting(&gpsdata, 500000);

        if (!ret) {
            // 240 tries at 0.5 seconds a try is a 2 minute timeout
            if (240 < wait_clicks++) {
                cout << "cgps: timeout contactong gpsd" << endl;
            }
        } 
        else {
            wait_clicks = 0;
            errno = 0;
            *message = '\0';
            if (gps_read(&gpsdata, message, sizeof(message)) == -1) {
                cout << "cgps: socket error " << endl;
            }

            gpsFix = update_gps_panel(&gpsdata);

            Serialize<GPSFix> gnss_data;
            std::stringstream ss;
            gnss_data.serialize(gpsFix, ss);
            ser_gnss = ss.str().data();
            gnss_publisher.Send(ser_gnss);


            Odometry gpsOdom;
            gpsOdom = gps_to_pose_conversion(gpsFix);

            Serialize<Odometry> gnss_odom;
            std::stringstream ss_odom;
            gnss_odom.serialize(gpsOdom, ss_odom);
            ser_odom = ss_odom.str().data();
            gps_odom_pub.Send(ser_odom);


        }

    }
}
