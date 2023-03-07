#include "vehicleio.hpp"
#include "commons.hpp"

using namespace cv;
vehicleIO::vehicleIO()
{   
    subscriber.reset(new eCAL::CSubscriber("ctrl"));
    subscriber->AddReceiveCallback(std::bind(&vehicleIO::receive_data, this, std::placeholders::_1, std::placeholders::_2));
    publisher.reset(new eCAL::string::CPublisher<std::string>("pix_feedback"));
    frame.reset(new ctrl);
}

float vehicleIO::absolute(float val,float least_count)
{
    float mod_val = fmod(val,least_count);
    if(mod_val<=(least_count/2))
        return (val-mod_val);
    else
        return val+(least_count-mod_val);
}

float vehicleIO::input_parse(float rec_val,float signal_min,float signal_max,float adc_min,float adc_max)
{
    float left_pulse = signal_max - signal_min;
    float right_pulse = adc_max - adc_min;

    float valueScaled = (float)(rec_val - signal_min) / (float)left_pulse;
    return adc_min + (valueScaled * right_pulse);
}

void vehicleIO::receive_data(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
{
    char* buffer=static_cast<char*>(data_->buf);
    ctrl rec_data = parse_data(buffer);

    frame->linear_v = rec_data.linear_v;
    frame->steer = rec_data.steer;
    frame->time = rec_data.time;
    std::cout << std::setprecision(8) << std::fixed << std::endl;
}

ctrl vehicleIO::parse_data(char* buffer)
{
    std::stringstream ss(buffer);
    Serialize<ctrl> IPC;
    ctrl data;
    data = IPC.deserialize(ss,data);
    return data;
}

float vehicleIO::check(float val,float MAX_VAL)
{
    if(val>MAX_VAL){
        // error slow the the car
        return 0;
    }
    return val;
}
cv::Mat blueImage(1000, 1000, CV_8UC3, Scalar(255, 255, 255));
void vehicleIO::debug()
{
    // double throttle_input;
    // std::cin>>throttle_input;
    // float output_val = input_parse(throttle_input,0,100,0,255);
    // std::cout<<"throttle input -> "<< throttle_input <<"  "<< output_val<<std::endl;

    int input_target = 0;
    double correct_val = pid(input_target,feedback.linear_v);
    correct_val = input_parse(correct_val,)


}



void vehicleIO::default_values_to_pix()
{
    // handbrake release
    parkTarget = 0;
    // forward gear engaged
    gearTarget = 4; 
}

void vehicleIO::Pid


void vehicleIO::send_to_pix()
{            

    while(true)
    {
    sleep(0.02);
    auto timenow =std::chrono::high_resolution_clock::now();
    auto time_nano = (std::chrono::duration_cast<std::chrono::nanoseconds>(timenow.time_since_epoch()).count())/1e9;
    std::shared_ptr<ctrl> new_frame = frame;
    
    if(time_nano-(new_frame->time) >= 0.1)
    {
        throttle_value = 0;
        steering_angle = 0;
        parkTarget = 1;
        init_flag = false;
        std::cout << "................Entered in to Default....... "<< new_frame->time << std::endl;
    }
    else
    {   
        throttle_value = new_frame->linear_v;
        steering_angle = new_frame->steer;
        switch(init_flag)
        {
            case(0):
                throttle_value = 0;
                steering_angle = 0;
                default_values_to_pix();
                init_flag = true;
                break;
            case(1):
                throttle_value = check(new_frame->linear_v,3);
                steering_angle = new_frame->steer;
                break;
        }
    }

    if (socket_can_write.open("can0") == scpp::STATUS_OK)
    {   
        
        // park disengaged
        while(write_park(parkTarget) !=  scpp::STATUS_OK)
            write_park(parkTarget);

        // // forward gear engaged
        while(write_gear(gearTarget) !=  scpp::STATUS_OK)
            write_gear(gearTarget);

        // // vcu default values
        while(write_vcu() !=  scpp::STATUS_OK)
            write_vcu();
        
        // throttle value send
        throttle_value = absolute(throttle_value,least_count_throttle);
        std::cout <<" Throttle "<< throttle_value << " ";
        while(write_throttle(throttle_value) !=  scpp::STATUS_OK)
            write_throttle(throttle_value);
                
        // steer value send
        steering_angle = absolute(steering_angle, least_count_steering);
        std::cout <<" Steer "<< steering_angle << " ";
        while(write_steering_angle(steering_angle) !=  scpp::STATUS_OK)
            write_steering_angle(steering_angle);
    }
    //debug this
    socket_can_write.close();
    }
}

bool vehicleIO::write_steering_angle(float steering_angle)
{
    Steering_Command.data[0] = steerEnCtrl;
    Steering_Command.data[1] = steerAngleSpeed;
    Steering_Command.data[2] = 0;
    float steering_send_initial = round(input_parse(steering_angle,-30,30,0,steerMaxVal));
    Steering_Command.data[3] = steering_send_initial / 255;
    Steering_Command.data[4] = ((steering_send_initial / 255) - int((steering_send_initial / 255))) * 255;
    Steering_Command.data[5] = 0;
    Steering_Command.data[6] = 0;
    Steering_Command.data[7] = steerCheckSum;

    scpp::CanFrame cf_to_write_steer;
    cf_to_write_steer.id = t_steerCanID;
    cf_to_write_steer.len = 8;
    for (int i = 0; i < 8; ++i)
        cf_to_write_steer.data[i] = Steering_Command.data[i];
    sleep(0.02);
    auto write_steering_status = socket_can_write.write(cf_to_write_steer);
    return write_steering_status;

}

bool vehicleIO::write_throttle(float throttle_value)
{
    Throttle_Command.data[0] = driveEnCtrl;
    Throttle_Command.data[1] = driveAcc;
    Throttle_Command.data[2] = 0;
    Throttle_Command.data[3] = driveThrottlePedalTarget;
    Throttle_Command.data[4] = 0;
    int throttle_send_initial = round(input_parse(throttle_value,0,speedMaxLimit,0,ThrottleMaxVal));
    Throttle_Command.data[5] = throttle_send_initial;
    Throttle_Command.data[6] = 0;
    Throttle_Command.data[7] = driveCheckSum;

    scpp::CanFrame cf_to_write_throttle;
    cf_to_write_throttle.id = t_throttleCanID;
    cf_to_write_throttle.len = 8;
    for (int i = 0; i < 8; ++i)
        cf_to_write_throttle.data[i] = Throttle_Command.data[i];
    auto write_throttle_status = socket_can_write.write(cf_to_write_throttle);
    return write_throttle_status;
}

bool vehicleIO::write_park(float parkTarget)
{
    Park_Command.data[0] = 1;
    Park_Command.data[1] = parkTarget;
    Park_Command.data[2] = 0;
    Park_Command.data[3] = 0;
    Park_Command.data[4] = 0;
    Park_Command.data[5] = 0;
    Park_Command.data[6] = 0;
    Park_Command.data[7] = parkCheckSum;

    scpp::CanFrame cf_to_write_park;
    cf_to_write_park.id = t_parkCanID;
    cf_to_write_park.len = 8;
    for (int i = 0; i < 8; ++i)
        cf_to_write_park.data[i] = Park_Command.data[i];
    auto write_park_status = socket_can_write.write(cf_to_write_park);
    return write_park_status;
}

bool vehicleIO::write_gear(float gearTarget)
{
    Gear_Command.data[0] = gearEnCtrl;
    Gear_Command.data[1] = gearTarget;
    Gear_Command.data[2] = 0;
    Gear_Command.data[3] = 0;
    Gear_Command.data[4] = 0;
    Gear_Command.data[5] = 0;
    Gear_Command.data[6] = 0;
    Gear_Command.data[7] = gearCheckSum;

    scpp::CanFrame cf_to_write_gear;
    cf_to_write_gear.id = t_gearCanID;
    cf_to_write_gear.len = 8;
    for (int i = 0; i < 8; ++i)
        cf_to_write_gear.data[i] = Gear_Command.data[i];
    auto write_gear_status = socket_can_write.write(cf_to_write_gear);
    return write_gear_status;

}

bool vehicleIO::write_vcu()
{
    Vehicle_Command.data[0] = steerModeCtrl;
    Vehicle_Command.data[1] = driveModeCtrl;
    Vehicle_Command.data[2] = headlightCtrl;
    Vehicle_Command.data[3] = vehicleVINReq;
    Vehicle_Command.data[4] = 0;
    Vehicle_Command.data[5] = 0;
    Vehicle_Command.data[6] = 0;
    Vehicle_Command.data[7] = vehicleModeCheckSum;

    scpp::CanFrame cf_to_write_vcu;
    cf_to_write_vcu.id = t_vehicleModeCanID;
    cf_to_write_vcu.len = 8;
    for (int i = 0; i < 8; ++i)
        cf_to_write_vcu.data[i] = Vehicle_Command.data[i];
    auto write_vcu_status = socket_can_write.write(cf_to_write_vcu);
    return write_vcu_status;
}


FEEDBACK vehicleIO::feed_from_pix_control()
{
    std::cout<<"\n\nFeedback\n\n"<<std::endl;

    std::shared_ptr<ctrl> feed_frame = frame;
    
    FEEDBACK feedback;
    scpp::CanFrame fr;
    if (socket_can_read.open("can0") == scpp::STATUS_OK)
    {
    while(socket_can_read.read(fr) == scpp::STATUS_OK)
    {
        if(fr.id == r_steerCanID){
            int data = fr.data[3]*255 + fr.data[4];
            float steering_angle = input_parse(data,0,(steerMaxVal-1),-30,30);
            feedback.steering_angle = steering_angle;
        }
        if(fr.id == r_vcuCanID)
        {
            int data = fr.data[2]*255 + fr.data[3];
            if(data<(speedMaxVal+1000))
            {
                float vehicle_speed = input_parse(data,0,speedMaxVal,speedMin,speedMax);
                feedback.vehicle_speed = vehicle_speed;
            }
            else
            {
                float vehicle_speed = input_parse(data,255*255,speedMinVal,speedMin,(speedMax*(-1)));
                feedback.vehicle_speed = vehicle_speed;
            
            }  

        }
    }

    if(feedback.vehicle_speed > (feed_frame->linear_v)){
       std::cout<< "\n****************** Vehicle is running faster than it should*******************\n"<<std::endl;
    }
    //debug this
    socket_can_read.close();
    }
    return feedback;
}

void vehicleIO::send_to_control(FEEDBACK feed)
{
    Serialize<FEEDBACK> IPC_send;
    // std::cout << feed.vehicle_speed << ", " << feed.steering_angle << std::endl;
    std::stringstream ss;
    IPC_send.serialize(feed,ss);
    std::string temp = ss.str();
    publisher->Send(temp);
}