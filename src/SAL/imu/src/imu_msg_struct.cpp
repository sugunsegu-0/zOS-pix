#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <math.h>
#include <string.h>
// #include <yaml-cpp/yaml.h>

//Microstrain Libraries.
#include "mscl/mscl.h"
#include "../include/getCurrentConfig.h"
#include "../include/parseData.h"
#include "../include/setCurrentConfig.h"
#include "../include/startSampling.h"
#include "../include/setToIdle.h"

#include "localization/localizationDS.hpp"

//eCal for communication.
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

//Serialization for communication.
#include <commons.hpp>

//Message structures.
Header header;
Quaternion orientation;
Vector3 angular_velocity;
Vector3 linear_acceleration;
Imu imu;

using namespace std;

int main(int argc, char** argv){
    //Including the sensor configuration file.
    // YAML::Node config = YAML::LoadFile("../config/imu_config.yaml"); //${ROOTDIR}/imu/config/imu_config.yaml

    //Device port number and baudrate.
    // std::string port = config["port"].as<std::string>();
    // int baudrate = config["baudrate"].as<int>();

    //Get system time for timestamp.
    auto now = std::chrono::high_resolution_clock::now();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    // auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

    //eCal Publisher.
    eCAL::Initialize(argc, argv, "IMU Publisher");

    // Create a String Publisher that publishes on the topic "imu"
    eCAL::string::CPublisher<std::string> imu_publisher("imu/data");

    //For serialization of msg.
    std::string ser_imu;

    try
    {
        //Initiate the serial connection.
        mscl::Connection connection = mscl::Connection::Serial("/dev/ttyACM1", 115200);

        //Create a node with the connection.
        mscl::InertialNode node(connection);

        //Get device information.
        std::string modelName = node.modelName();
        
        //Start
        setToIdle(node);
        startSampling(node);

        while(eCAL::Ok()){ //TODO: add while condition checking status of IMU data stream
            //Get system time.
            now = std::chrono::high_resolution_clock::now();
            nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            // seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();


            //Create Header message structure.
            header.frame_id = "imu";

            mscl::MipDataPackets packets = node.getDataPackets(50);
            
             //Create IMU structure.
            for (auto packet : packets){

                packet.descriptorSet();
                packet.deviceTimestamp();

                mscl::MipDataPoints points = packet.data();
                for(mscl::MipDataPoint dataPoint : points)
                {
                    // std::cout << dataPoint.channelName() << std::endl;
                    // std::cout << dataPoint.storedAs() << std::endl;
                    
                    // std::cout << dataPoint.as_float() << std::endl;

                    //Acceleration
                    if(dataPoint.channelName() == "estLinearAccelX"){                              
                        // std::cout << "compAccelX: " << dataPoint.as_float() << std::endl;
                        linear_acceleration.x = dataPoint.as_float();
                    }
                    if(dataPoint.channelName() == "estLinearAccelY"){                                
                        // std::cout << "scaledAccelY: " << dataPoint.as_float() << std::endl;
                        linear_acceleration.y = dataPoint.as_float();
                    }
                    if(dataPoint.channelName() == "estLinearAccelZ"){                                
                        // std::cout << "scaledAccelZ: " << dataPoint.as_float() << std::endl;
                        linear_acceleration.z = dataPoint.as_float();
                    }
                    imu.linear_acceleration = linear_acceleration;

                    //Angular Velocity
                    if(dataPoint.channelName() == "estAngularRateX"){                             
                        // std::cout << "estAngularRateX: " << dataPoint.as_float() << std::endl;
                        angular_velocity.x = dataPoint.as_float();
                    }
                    if(dataPoint.channelName() == "estAngularRateY"){                              
                        // std::cout << "estAngularRateY: " << dataPoint.as_float() << std::endl;
                        angular_velocity.y = dataPoint.as_float();
                    }
                    if(dataPoint.channelName() == "estAngularRateZ"){                               
                        // std::cout << "estAngularRateZ: " << dataPoint.as_float() << std::endl;
                        angular_velocity.z = dataPoint.as_float();
                    }
                    imu.angular_velocity = angular_velocity;

                    //Orientation
                    if(dataPoint.channelName() == "estOrientQuaternion"){
                        auto q = dataPoint.as_Vector();
                        // const std::type_info& ti1 = typeid(q);
                        // std::cout << ti1.name() << std::endl;
                        // std::cout << q.as_doubleAt(0) << std::endl;
                        // std::cout << q.as_doubleAt(1) << std::endl;
                        // std::cout << q.as_doubleAt(2) << std::endl;
                        // std::cout << q.as_doubleAt(3) << std::endl;
                        // std::cout << "---" << std::endl;
                        orientation.x = q.as_doubleAt(0);
                        orientation.y = q.as_doubleAt(1);
                        orientation.z = q.as_doubleAt(2);
                        imu.orientation = orientation;
                    }

                    //Orientation Covariance.
                    if(dataPoint.channelName() == "estAttitudeUncertQuaternion"){
                        auto cov = dataPoint.as_Matrix();
                        // std::cout << cov.rows() << ", " << cov.columns() << std::endl;
                        // const std::type_info& ti1 = typeid(cov);
                        // std::cout << ti1.name() << std::endl;
                        
                        imu.orientation_covariance[0] = cov.as_doubleAt(0,0);
                        imu.orientation_covariance[1] = 0.0;
                        imu.orientation_covariance[2] = 0.0;
                        imu.orientation_covariance[3] = 0.0;
                        imu.orientation_covariance[4] = cov.as_doubleAt(0,1);
                        imu.orientation_covariance[5] = 0.0;
                        imu.orientation_covariance[6] = 0.0;
                        imu.orientation_covariance[7] = 0.0;
                        imu.orientation_covariance[8] = cov.as_doubleAt(0,2);
                    }                            

                }
                        // std::cout << "---" <<std::endl;

            }
           
            imu.header = header;
            imu.header.time = nanoseconds/1e9;
           
        //    std::cout << "JUST BEFORE SER compAccelx: "<< imu.linear_acceleration.x << std::endl;
           Serialize<Imu> imu_data;
           std::stringstream ss;
           imu_data.serialize(imu, ss);
           ser_imu = ss.str().data();
        //    std::cout << "JUST AFTER SER compAccelx: "<< ser_imu << std::endl;
           imu_publisher.Send(ser_imu);

            cout << std::setprecision(9) << std::fixed << endl;
            cout << "frame_id " << imu.header.frame_id << endl;
            cout << "stamp " << imu.header.time << endl;
            cout << "ax " << imu.linear_acceleration.x << endl;
            cout << "ay " << imu.linear_acceleration.y << endl;
            cout << "az" << imu.linear_acceleration.z << endl;
        }
        
        // finalize eCAL API
        eCAL::Finalize();
    }
    catch(mscl::Error& e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}