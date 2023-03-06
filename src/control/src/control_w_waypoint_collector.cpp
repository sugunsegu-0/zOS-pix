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

list<double> path;
ctrl ctrl_cmd;

bool first_f = true;
float prev_yaw = 0.0;
float target_speed_max = 2 /3.6;
float target_speed_min = 1.5 /3.6;

struct two_points
{
    cv::Point a;
    cv::Point b;
};
two_points getLFCpoint(cv::Point start, float m, float l)
{
    two_points out;
    cv::Point a, b;
    // slope is 0
    if (m == 0)
    {
        a.x = start.x + l;
        a.y = start.y;
 
        b.x = start.x - l;
        b.y = start.y;
    }
 
    // if slope is infinite
    else if (m == std::numeric_limits<float>::max())
    {
        a.x = start.x;
        a.y = start.y + l;
 
        b.x = start.x;
        b.y = start.y - l;
    }
    else
    {
        float dx = (l / sqrt(1 + (m * m)));
        float dy = m * dx;
        a.x = start.x + dx;
        a.y = start.y + dy;
        b.x = start.x - dx;
        b.y = start.y - dy;
    }
    out.a = a;
    out.b = b;
    return out;
}
void save_waypoints() {
    
    ofstream path_file("/home/mz/Documents/sugun_zos/zOS-pix/src/control/src/tracked_path.txt", std::ofstream::out);

    cout << "saved" << endl;
    if (path_file.is_open())
    {
      for (auto it = path.begin(); it != path.end(); ++it)
        path_file << *it << "\n";
        

      path_file.close();
    }
    
}

float findAngle(double M1, double M2)
{
    // Store the tan value  of the angle
    double angle = abs((M2 - M1)
                        / (1 + M1 * M2));

    // Calculate tan inverse of the angle
    double ret = atan(angle);

    // Convert the angle from
    // radian to degree
    double val = (ret * 180) / PI;
    // cout << ret << " rad, " << val <<" deg" << endl;

    return ret;
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
    eCAL::string::CSubscriber<std::string> vehicle{"vehicle-feed"};
    State(float yaw_in=0,float v_in=0)
    {
        yaw=yaw_in;
        v=v_in;
        gps_odom_sub.AddReceiveCallback(std::bind(&State::Update,this,std::placeholders::_2));
        vehicle.AddReceiveCallback(std::bind(&State::plot_feed,this,std::placeholders::_2));

    }
    void theroy(float vel,float delta)
    {
        // x += vel * cos(yaw) * dt;
        // y += vel * sin(yaw) * dt;
        yaw += vel / WB * tan(delta) * dt;
        v =  vel;
        rear_x = x - ((WB / 2) * cos(yaw));
        rear_y = y - ((WB / 2) * sin(yaw));

    }
    
    void plot_feed(const std::string &msg)
    {   
        std::cout<< "my code"<<std::endl;
        std::vector<double> in;
        std::vector<double> out;
        std::stringstream ss_fe_in(msg);
        Serialize<FEEDBACK> data;
        FEEDBACK out_data;
        out_data = data.deserialize(ss_fe_in,out_data);
        cv::Mat blueImage(1000, 1000, CV_8UC3, Scalar(255, 255, 255));
        cout << ".........in" << endl;
        if(in.size()>999)
        {
            in.erase(in.begin());
            in.push_back(v);
            out.erase(out.begin());
            out.push_back(out_data.vehicle_speed); 
        }
        else
        {
            in.push_back(v);
            out.push_back(out_data.vehicle_speed);
        }

        std::vector<cv::Point> pts;
        std::vector<cv::Point> pts_1;
        int i=0;
        for(auto In:in)
        {
            pts.push_back(cv::Point(i,500+In*10));
            i++;
        }
        i=0;
        for(auto ou:out)
        {
            pts_1.push_back(cv::Point(i,500+ou*10));
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
        
        // cout << "callback: " << "x: " << odom.pose.pose.position.x << "y: " << odom.pose.pose.position.y << endl;

        // x += vel * cos(yaw) * dt;
        x = odom.pose.pose.position.x;
        // y += vel * sin(yaw) * dt;
        y = odom.pose.pose.position.y;
        // if (gps_init){
        //     yaw = atan2(y - prev_y, x - prev_x);
        // }
        // else{
        //     yaw = 1.57;
        // }
        
        // yaw += vel / WB * tan(delta) * dt;
        // v = odom.twist.twist.linear.x ;
        // prev_x = x;
        // prev_y = y;
        // rear_x = prev_x;
        // rear_y = prev_y;
        // gps_odom_sub.AddReceiveCallback(std::bind(&State::Update, this,std::placeholders::_2));
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

class States
{
    
    private:
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> yaw;
    
    std::vector<float> v;
    std::vector<float> t;
    public:
    void append(float t_in,State* state)
    {
        x.push_back(state->x);
        y.push_back(state->y);
        yaw.push_back(state->yaw);
        v.push_back(state->v);
        t.push_back(t_in);
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
            
            // k = 1;
            // Lfc = 1;

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
    }
    



std::vector<float> quaternion_from_euler(float roll,float pitch,float yaw)
{
    
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    std::vector<float> q;
    q.push_back(cy * cp * cr + sy * sp * sr);
    q.push_back(cy * cp * sr - sy * sp * cr);
    q.push_back(sy * cp * sr + cy * sp * cr);
    q.push_back(sy * cp * cr - cy * sp * sr);

    return q;
}


int main(int argc, char * argv[])
{     

    eCAL::Initialize(argc, argv, "control");

    


    eCAL::string::CPublisher<std::string> out_control{"ctrl"};

    ifstream inFile;
    inFile.open("/home/mz/Documents/sugun_zos/zOS-pix/src/SAL/gps/src/path.txt");
    int count = 0; 
    double val;

    cout << std::setprecision(2) << std::fixed << endl;  // set output to fixed floating point, 8 decimal precision

    
    std::vector<float> cx;
    std::vector<float> cy;
    std::vector<float> cyaw;
    std::vector<float> curvature;
    eCAL::CSubscriber odom_sub{"gps/odom"};
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
    cout << "len of x array " << cx.size() << endl;
    cout << "len of y array " << cy.size() << endl;

    double ref_m = atan2(cy[1] - cy[0], cx[1] - cx[0]);
    // cout << ref_m << endl;
    // cv::Mat blueImage(1000, 1000, CV_8UC3, Scalar(255, 255, 255));
    
    // for (int i = 0; i < cx.size(); i++) {
    //     cyaw.push_back(atan2(cy[i+1] - cy[i], cx[i+1] - cx[i]));
    //     circle(blueImage, cv::Point(cx[i]*8+300,cy[i]*8+150), 5, Scalar(0, 0, 0), -1, LINE_8);

    // }

    for (int i = 0; i < cyaw.size()-7; i++) {
        auto cumum_sum =0.0;
        for(int j = 0 ; j<5; j++)
            {
                auto diff= atan2(cy[i+j+2] - cy[i+j+1], cx[i+j+2] - cx[i+j+1]) - atan2(cy[i+j+1] - cy[i+j], cx[i+1+j] - cx[i+j]);
                cumum_sum=cumum_sum+abs(diff);
            }
        // cout << cumum_sum/5 << endl;
        curvature.push_back(cumum_sum/5);
    }

    float T = 100.0;
    float target_speed = 0.0; 
    int lastIndex = cx.size() - 1;
    float time = 0.0;

    State state = State(ref_m, 0.0); // state(x,y,yaw,v)
    
    States states;
    states.append(time, &state);
    

    TargetCourse target_course = TargetCourse(cx, cy);
    std::pair<int,float> out = target_course.search(&state);

    int index = 0;

    cout << "lastindex" << lastIndex << endl;
    
    while(eCAL::Ok() &&  lastIndex-6 != index)
    {
        
        std::this_thread::sleep_for(std::chrono::milliseconds(rest));
        if(gps_init)
        {
            std::pair<float,float> out_last = pure_pursuit_steer_control(&state, target_course, out.first);

            float cur_yaw = curvature[index];
            // cout << "................index" << index << "index" << cur_yaw << endl;
            target_speed  = (target_speed_max - (cur_yaw/0.2) * (target_speed_max - target_speed_min));
            state.v = target_speed;

            time += dt;
            states.append(time, &state);

            std::pair<int,float> out = target_course.search(&state);
            index = out.first;
            two_points poin = getLFCpoint(cv::Point(state.x,state.y), cur_yaw, target_course.Lf);
            // cout<<"lf"<<target_course.Lf<<endl;
            // circle(blueImage, cv::Point(state.x*8+300,(state.y*8+150)), 5, Scalar(255, 0, 0), -1, LINE_8);
            // circle(blueImage, cv::Point(poin.a.x*20+750,(poin.a.y*20+250)), 5, Scalar(0, 0, 255), -1, LINE_8);
            // circle(blueImage, cv::Point(poin.b.x*20+750,(poin.b.y*20+250)), 5, Scalar(255, 0, 0), -1, LINE_8);
            state.theroy(target_speed,out_last.first);
            // Control Command Publish
            ctrl_cmd.linear_v = target_speed * 3.6;   // kmph
            ctrl_cmd.steer =  out_last.first * 180 / 3.14159265 ;   // degs

            auto now = std::chrono::high_resolution_clock::now();
            auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            
            ctrl_cmd.linear_v = 1.0;   // kmph
            ctrl_cmd.steer = 1.0 ;   // degs
            ctrl_cmd.time = nanoseconds/1e9;
            cout << "Command: speed " << ctrl_cmd.linear_v << ", steer " << ctrl_cmd.steer << "Lahead" << target_course.Lf << endl; 
            Serialize<ctrl> data;
            std::stringstream ss;
            data.serialize(ctrl_cmd,ss);
            std::string temp = ss.str().data();
            out_control.Send(temp.c_str(), sizeof(temp));
            // cv::imshow("test",blueImage);
            // cv::waitKey(1);
        }
        

    }
    cv::destroyAllWindows();
    cout << "finished" << endl;
    // Publishing zero sped and zero steer at the end 
    double final_speed = 0.0;
    double final_steer = 0.0;

    // state->Update(final_speed, final_steer);

    auto now = std::chrono::high_resolution_clock::now();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    
    for (int i = 0; i < cyaw.size()-7; i++) {
        ctrl_cmd.linear_v = 0.0;   // kmph
        ctrl_cmd.steer = 0.0;      // degs
        ctrl_cmd.time = nanoseconds/1e9;
        Serialize<ctrl> data;
        std::stringstream ss;
        data.serialize(ctrl_cmd,ss);
        std::string temp = ss.str().data();
        out_control.Send(temp.c_str(), sizeof(temp));
        cout << "Command: speed " << ctrl_cmd.linear_v << ", steer " << ctrl_cmd.steer << endl; 
    }
    save_waypoints();

    // finalize eCAL API
    eCAL::Finalize();
   
  return 0;
}