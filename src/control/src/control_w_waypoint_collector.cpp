#include <iostream>
#include <math.h>
#include <limits>
#include <algorithm>
#include <sys/time.h>
#include <string>
#include <cstdlib>
#include <chrono>
#include "motion-planning/motion-planningDS.hpp"
#include <memory>
#include <string>
#include "commons.hpp"
// #include "./data-structures/include/localization/localizationDS.hpp"
#include "../../data-structures/include/localization/localizationDS.hpp"
#include <signal.h>
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
using namespace std;


// IPC

#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>
#include <iostream>
#include <thread>

#include "control/control-DS.hpp"
#include "vehicleio/vehicle_DS.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


#define PI 3.14159265;

using namespace cv;
float k = 1;
float Lfc = 0.75;  

float frequency=30;
float dt = 1/frequency; 
float WB = 1.93; 
int rest = 1000/frequency;
bool gps_init = false;

bool visualize_init = false;
cv::Mat blueImage(1000, 1000, CV_8UC3, Scalar(255, 255, 255));


list<double> path;

bool first_f = true;
float prev_yaw = 0.0;
float target_speed_max = 5 /3.6;
float target_speed_min = 1.5 /3.6;
float max_curvature;
float min_curvature;


std::vector<float> cx;
std::vector<float> cy;
std::vector<float> cyaw;
std::vector<float> curvature;
int lastIndex;
int index_path = 0;
std::string path_for_waypoints = "/home/minuszero/dev/zOS-pix/src/SAL/gps/src/path.txt";



std::string control_to_vehicleio(float speed, float steer, int cur_index)
{
    ctrl ctrl_cmd;
    auto now = std::chrono::high_resolution_clock::now();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    ctrl_cmd.time = nanoseconds/1e9;
    if (lastIndex > cur_index){
        ctrl_cmd.linear_v = speed * 3.6;   // kmph
        ctrl_cmd.steer =  steer * 180 / 3.14159265 ;   // degs
    }
    else{
        ctrl_cmd.linear_v = 0.0;
        ctrl_cmd.steer = 0.0;
    }
    
    // only for testing
    // ctrl_cmd.linear_v = 1.0; // kmph
    // ctrl_cmd.steer = 0.0; // degs

    cout << "Command: speed " << ctrl_cmd.linear_v << ", steer " << ctrl_cmd.steer << endl; 
    Serialize<ctrl> data;
    std::stringstream ss;
    data.serialize(ctrl_cmd,ss);
    std::string temp = ss.str().data();

    return temp;

}

void visualize(float curr_x, float curr_y)
{
    if (!visualize_init){
        for (int i = 0; i < cx.size(); i++) {
            circle(blueImage, cv::Point(cx[i]*8+300,cy[i]*8+150), 5, Scalar(0, 0, 0), -1, LINE_8);
        }
        visualize_init = true;
    }
    
    circle(blueImage, cv::Point(curr_x*8+300,(curr_y*8+150)), 5, Scalar(255, 0, 0), -1, LINE_8);
    cv::imshow("test",blueImage);
    cv::waitKey(1);

}

void load_waypoints()
{
    ifstream inFile;
    inFile.open(path_for_waypoints);
    int count = 0; 
    double val;

    if (!inFile) {
        cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }
    while (inFile >> val) {
        if ((count%2) == 0){
            cx.push_back(val);
        }
        else{
            cy.push_back(val);
        }
        count = count + 1;
    }
    inFile.close();

    for (int i = 0; i < cx.size(); i++) {
        cyaw.push_back(atan2(cy[i+1] - cy[i], cx[i+1] - cx[i]));
    }

    for (int i = 1; i < cyaw.size()-7; i++) {
        auto cumum_sum =0.0;
        for(int j = 0 ; j<5; j++)
            {
                auto diff= atan2(cy[i+j+2] - cy[i+j+1], cx[i+j+2] - cx[i+j+1]) - atan2(cy[i+j+1] - cy[i+j], cx[i+1+j] - cx[i+j]);
                cumum_sum=cumum_sum+abs(diff);
            }
        curvature.push_back(cumum_sum/5);
        cout << cumum_sum/5 << endl;
    }
    max_curvature = *max_element(curvature.begin(), curvature.end());
    min_curvature = *min_element(curvature.begin(), curvature.end());
}

class State
{
    public:
    float x;
    float y;
    float yaw;
    float v;
    float rear_x;
    float rear_y;
    float prev_y = 0.0;
    float prev_x = 0.0;

    eCAL::string::CSubscriber<std::string> gps_odom_sub{"gps/odom"};
    eCAL::string::CSubscriber<std::string> vehicle_feed_sub{"pix_feedback"};

    State(float yaw_in=0,float v_in=0)
    {
        yaw=yaw_in;
        v=v_in;
        gps_odom_sub.AddReceiveCallback(std::bind(&State::Update,this,std::placeholders::_2));
        vehicle_feed_sub.AddReceiveCallback(std::bind(&State::plot_feed,this,std::placeholders::_2));
    }

    void theory(float vel,float delta)
    {
        x += vel * cos(yaw) * dt;
        y += vel * sin(yaw) * dt;
        yaw += vel / WB * tan(delta) * dt;
        v =  vel;
        rear_x = x - ((WB / 2) * cos(yaw));
        rear_y = y - ((WB / 2) * sin(yaw));
    }
    
    void plot_feed(const std::string &msg)
    {   
        std::vector<double> in;
        std::vector<double> out;
        std::stringstream ss_fe_in(msg);
        Serialize<FEEDBACK> data;
        FEEDBACK out_data;
        out_data = data.deserialize(ss_fe_in,out_data);
        cv::Mat blueImage(1000, 1000, CV_8UC3, Scalar(255, 255, 255));

        if(in.size()>999)
        {
            in.erase(in.begin());
            in.push_back(v);
            out.erase(out.begin());
            out.push_back(out_data.vehicle_speed); 
        }
        else
        {
            cout << out_data.vehicle_speed << endl;
            in.push_back(v);
            out.push_back(out_data.vehicle_speed);
        }

        std::vector<cv::Point> pts;
        std::vector<cv::Point> pts_1;
        int i=0;
        for(auto In:in)
        {
            pts.push_back(cv::Point(i,500+In*100));
            i++;
        }
        i=0;
        for(auto ou:out)
        {
            pts_1.push_back(cv::Point(i,500+ou*100));
            i++;
        }

        cv::polylines(blueImage,pts,false,cv::Scalar(0,0,255),1,LINE_8);
        cv::polylines(blueImage,pts_1,false,cv::Scalar(0,255,0),1,LINE_8);    

        cv::imshow("Graph Plot", blueImage);
        cv::waitKey(1);
    }

    void Update(const std::string &msg)
    {
        std::stringstream ss_odom_in(msg);
        Serialize<Odometry> data;
        Odometry odom;
        odom = data.deserialize(ss_odom_in,odom);
        
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        // yaw += vel / WB * tan(delta) * dt;

        rear_x = x - ((WB / 2) * cos(yaw));
        rear_y = y - ((WB / 2) * sin(yaw));

        path.push_back(x);
        path.push_back(y);
        
        gps_init = true;
    }

    float distance(float point_x,float point_y)
    {
        float dx = rear_x - point_x;
        float dy = rear_y - point_y;
        float dis = sqrt(pow(dx,2)+pow(dy,2));
        return dis;
    } 

};

class TargetCourse
{
    public:
    std::vector<float> cx;
    std::vector<float> cy;
    int old_nearest_point_index;
    TargetCourse(std::vector<float> c_x,std::vector<float> c_y)
    {
        cx=c_x;
        cy=c_y;
        old_nearest_point_index = -1;
    }
    float Lf;
    std::pair<int,float> search(State* state) 
    {
        int ind;

        if (old_nearest_point_index==-1)
        {
            std::vector<float> dx;
            std::vector<float> dy;
            std::vector<float> d;
            
            for(auto icy:cy)
                dy.push_back(state->rear_y-icy);
            for(auto icx:cx)
                dx.push_back(state->rear_x-icx);

            for(int i = 0; i<dx.size();i++)
                d.push_back(sqrt(pow(dx[i],2)+pow(dy[i],2)));
            ind = *min_element(d.begin(), d.end());
            
            old_nearest_point_index = ind;
            cout << old_nearest_point_index << endl;
        }
        else
        {
            ind = old_nearest_point_index;
            float distance_this_index = state->distance(cx[ind],cy[ind]);
            while(true)
            {
                float distance_next_index = state->distance(cx[ind + 1],cy[ind + 1]);
                if(distance_this_index < distance_next_index)
                    break;
                if((ind + 1) < cx.size()) 
                    ind = ind + 1;
                distance_this_index = distance_next_index;
            }
            old_nearest_point_index = ind;
            
            k = 0.3;
            Lfc = 0.95;

            Lf = k * state->v + Lfc; 
           
            while(Lf > state->distance(cx[ind], cy[ind]))
            {
                if ((ind + 1) >= cx.size())
                    break;
                ind += 1;
            }
            
        }
        std::pair<int,float> out{ind,Lf};
        return out;
    }
};

std::pair<float,float> pure_pursuit_steer_control(State* state,TargetCourse trajectory,int pind) 
{
    std::pair<int,float> out = trajectory.search(state);

    int ind = out.first;
    float Lf = out.second;
    float tx;
    float ty;

    if(pind >= ind)
        ind = pind;

    if(ind < trajectory.cx.size())
    {
        tx = trajectory.cx[ind];
        ty = trajectory.cy[ind];
    }
    else
    {
        tx = trajectory.cx[-1];
        ty = trajectory.cy[-1];
        ind = trajectory.cx.size() - 1;
    }

    float alpha = atan2(ty - state->rear_y, tx - state->rear_x) - state->yaw;

    float delta = atan2(2.0 * WB * sin(alpha) / Lf, 1.0);
    std::pair<float,float> out_all{delta,ind};
    return out_all;
};
    


int main(int argc, char * argv[])
{     

    cout << std::setprecision(2) << std::fixed << endl;  // set output to fixed floating point, 8 decimal precision

    eCAL::Initialize(argc, argv, "control");
    eCAL::string::CPublisher<std::string> out_control{"ctrl"};

    // read recorded waypoints 
    load_waypoints();


    //Initialize State
    double ref_m = atan2(cy[1] - cy[0], cx[1] - cx[0]);
    float initial_speed = 0.0; 
    State state = State(ref_m, initial_speed); // State(x,y,yaw,v)
         
    // Initialize target course
    TargetCourse target_course = TargetCourse(cx, cy);
    std::pair<int,float> out = target_course.search(&state);


    lastIndex = cx.size() - 1;
    
    while(eCAL::Ok() &&  lastIndex-6 != index_path)
    {
        // maintain hz
        std::this_thread::sleep_for(std::chrono::milliseconds(rest));
        if(gps_init)
        {
            // Calculate Target Steer from pure_pursuit
            std::pair<float,float> out_last = pure_pursuit_steer_control(&state, target_course, out.first);

            // Calculate Target Speed from curvature of recorded path
            float cur_yaw = curvature[index_path];
            state.v  = (target_speed_max - (cur_yaw/(max_curvature-min_curvature)) * (target_speed_max - target_speed_min));

            std::pair<int,float> out = target_course.search(&state);
            index_path = out.first;

            // run theoritical ubdate for yaw calculation
            state.theory(state.v,out_last.first);

            // Publish control to vehicle io
            auto pub = control_to_vehicleio(state.v,out_last.first, out.first);
            out_control.Send(pub.c_str(), sizeof(pub));
            
            // Visualize Pure Pursuit Tracking 
            visualize(state.x, state.y);
        }
    }

    cv::destroyAllWindows();

    // finalize eCAL API
    eCAL::Finalize();
   
  return 0;
}