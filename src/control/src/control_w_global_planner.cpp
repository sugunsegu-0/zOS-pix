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


// IPC

#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>
#include <iostream>
#include <thread>


#include "control/control-DS.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;



float k = 0.05;
float Lfc = 2.0;  
float Kp = 1.0; 
float dt = 0.1; 
float WB = 0.5; 
class State
{
    
    
    public:
    float x;
    float y;
    float yaw;
    float v;
    float rear_x;
    float rear_y;
    State(float x_in=0,float y_in=0,float yaw_in=0,float v_in=0)
    {
        x=x_in;   
        y=y_in;
        yaw=yaw_in;
        v=1;
        rear_x= x - ((WB / 2) * cos(yaw));
        rear_y= y - ((WB / 2) * sin(yaw));

    }
    void Update(float a,float delta)
    {
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
        yaw += v / WB * tan(delta) * dt;
        v += a * dt;
        rear_x = x - ((WB / 2) * cos(yaw));
        rear_y = y - ((WB / 2) * sin(yaw));
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
    void append(float t_in,State state)
    {
        x.push_back(state.x);
        y.push_back(state.y);
        yaw.push_back(state.yaw);
        v.push_back(state.v);
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
    std::pair<int,float> search(State state) 
    {
        int ind;
        float Lf;
        if (old_nearest_point_index==-1)
        {
            std::vector<float> dx;
            std::vector<float> dy;
            std::vector<float> d;
            
            for(auto icy:cy)
                dy.push_back(state.rear_y-icy);
            for(auto icx:cx)
                dx.push_back(state.rear_x-icx);

            
            for(int i = 0; i<dx.size();i++)
                d.push_back(sqrt(pow(dx[i],2)+pow(dy[i],2)));
            ind = *min_element(d.begin(), d.end());
            std::cout << "output\n";
            
            old_nearest_point_index = ind;
        }
        else
        {
            ind = old_nearest_point_index;
            float distance_this_index = state.distance(cx[ind],cy[ind]);
            while(true)
            {
                float distance_next_index = state.distance(cx[ind + 1],cy[ind + 1]);
                if(distance_this_index < distance_next_index)
                    break;
                if((ind + 1) < cx.size()) 
                    ind = ind + 1;
                distance_this_index = distance_next_index;
            }
            old_nearest_point_index = ind;

            Lf = k * state.v + Lfc; 

           
            while(Lf > state.distance(cx[ind], cy[ind]))
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
std::pair<float,float> pure_pursuit_steer_control(State state,TargetCourse trajectory,int pind) 
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

        float alpha = atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw;

        float delta = atan2(2.0 * WB * sin(alpha) / Lf, 1.0);
        std::pair<float,float> out_all{delta,ind};
        return out_all;
    }
float proportional_control(float target,float current)
    {
        float a = Kp * (target - current);

        return a;
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
class purepursuit 
{
public:
    eCAL::CSubscriber path_sub{"path"};
    eCAL::CPublisher out_control{"ctrl"};  
    purepursuit()
    {
        path_sub.AddReceiveCallback(std::bind(&purepursuit::steer, this,std::placeholders::_1, std::placeholders::_2));
        
    }
    
   
    void steer(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)   
    {
        std::vector<float> cx;
        std::vector<float> cy;
        std::vector<float> angle;
        char* buffer=static_cast<char*>(data_->buf);
        std::stringstream ss(buffer);
        Serialize<path> IPC;
        path data;
        data = IPC.deserialize(ss,data);
        
        cx=data.x;
        cy=data.y;
        float T = 100.0;
        
        float target_speed = data.v[data.v.size()-1];
        State state = State(0.0,0.0,1.5707,0.0);
        
        int lastIndex = cx.size() - 1;
        float time = 0.0;
        States states;
        
        states.append(time, state);
        
        TargetCourse target_course = TargetCourse(cx, cy);
        std::cout<<"delta :"<<std::endl;
        std::pair<int,float> out = target_course.search(state);
       
        std::cout<<"delta :"<<std::endl;
        while(T >= time && lastIndex > out.first)
        {
            float ai = proportional_control(target_speed, state.v);
            std::pair<float,float> out_last = pure_pursuit_steer_control(state, target_course, out.first);

            state.Update(ai, out_last.first);

            time += dt;
            states.append(time, state);
            ctrl out;
            out.linear_v = ai;
            out.steer = out_last.first;
            Serialize<ctrl> data;
            std::stringstream ss;
            data.serialize(out,ss);
            std::string temp = ss.str();
            out_control.Send(temp.c_str(), sizeof(temp));


           

        }
        

    }


    
    


};


int main(int argc, char * argv[])
{
    purepursuit purepurObj;

    eCAL::Initialize(argc, argv, "control");
   
    
    // Just don't exit
    while (eCAL::Ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // finalize eCAL API
    eCAL::Finalize();
   
    
    
  return 0;
}