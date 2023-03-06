// c++
#include <vector>
#include <iostream>
#include <limits>
#include <sys/time.h>
#include <string>
#include <vector>
#include <cstdlib>

// openCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


// Algo Headers
#include "cubic_spline.h"
#include "frenet_path.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"

// ECAL Headers
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>
#include <iostream>
#include <thread>

// TODO Please set the PATHS
#include "commons.hpp"
#include "mapping/mappingDS.hpp"
#include "global_planner.hpp"
#include "global_points.hpp"
#include "localization/localizationDS.hpp"
#include "motion-planning/motion-planningDS.hpp"
using namespace std;

// TODO Use standard name space
using namespace robotics;
using namespace cv;

struct waypoint
{
    Vec_f wx;
    Vec_f wy;
};

std::string FSM_STATES[4] = {"vechicle_following", "emergency_breaking", "over_taking", "crusing"};
std::string current_state = "vechicle_following";

Vec_f wx{0.0, 0.0};
Vec_f wy{0.0, 0.0};

using std::placeholders::_1;

class motion_planinng
{
private:
    waypoint updated_way;
    
    /* Global variables can be defined here */

    #define SIM_LOOP 5000
    #define MAX_SPEED 50.0 / 3.6          // maximum speed [m/s]
    #define MAX_CURVATURE 300.0           // maximum curvature [1/m]
    #define D_ROAD_W 10.0                 // road width sampling length [m]
    #define DT 0.4                        // time tick [s]
    #define MAXT 40.0                     // max prediction time [m]
    #define MINT 4.0                      // min prediction time [m]
    #define D_T_S 5.0 / 3.6               // target speed sampling length [m/s]
    #define N_S_SAMPLE 1                  // sampling number of target speed
    #define ROBOT_RADIUS 20               // robot radius [m]
    #define KJ 0.
    #define KT 0.1
    #define KD 1
    #define KLAT 1
    #define KLON 1
    #define Scale 2
    uint64_t var = 0;
    double MAX_ACCEL = 20.0;              // maximum acceleration [m/ss]
    double MAX_ROAD_WIDTH_L;              // maximum road width [m]
    double MAX_ROAD_WIDTH_R;
    double TARGET_SPEED = 40.0 / 3.6;     // target speed [m/s]

    float c_speed = 10.0 / 3.6;
    float c_d = 2.0;
    float c_d_d = 0.0;
    float c_d_dd = 0.0;
    float s0 = 0.0;

    FrenetPath final_path;
    FrenetPath fp;
    // FrenetPath fp_bot;
    Vec_f r_x;
    Vec_f r_y;
    Vec_f ryaw;
    Vec_f rcurvature;
    Vec_f rs;
    void Behavior_planner(std::string current_state, int flag, double vehicle_density) // add_class_based
    {
        if (vehicle_density > 40)
        {
            TARGET_SPEED = 20;
            MAX_ACCEL = 10;
            current_state = "CRUISING";
        }
        if (vehicle_density > 30)
        {
            TARGET_SPEED = 20;
            MAX_ACCEL = 30;
            current_state = "OVER TAKING";
        }
        if (vehicle_density > 20)
        {
            TARGET_SPEED = 10;
            MAX_ACCEL = 5;
            current_state = "VEHICLE FOLLOWING";
        }
        if (trigger == 1)
        {
            TARGET_SPEED = 10;
            MAX_ACCEL = 20;
            current_state = "TURNING";
        }
        if (vehicle_density > 15 or flag == 1)
        {
            TARGET_SPEED = 0;
            MAX_ACCEL = 50;
            current_state = "E-STOP";
        }
    }

    std::vector<Poi_f> obstcles
    {
        {{0.0, 0.0}}
        
    };

    std::vector<cv::Point> lane1;
    std::vector<cv::Point> lane2;

public:
    eCAL::CSubscriber obj_sub{"obj"};
    eCAL::CSubscriber lane_sub{"lanes"};
    eCAL::CSubscriber free_sub{"free"};
    eCAL::CSubscriber local_sub{"free"};
    eCAL::CPublisher path_pub{"path"};
    

// TODO Access GPS Waypoints from other file ----------------------------------------------

    std::vector<std::vector<double>> pos_to_track = {{3.79838e-11, -0.0221404},
                                                     {514.258, -48.2664}};

//------------------------------------------------------------------------------------

    int trigger = 0;
    std::map<int, instructions> global_waypoints = get_global_waypoints(pos_to_track);
    
    // TODO generate waypoints in another file
    motion_planinng()
    {
        obj_sub.AddReceiveCallback(std::bind(&motion_planinng::Update_weight, this,std::placeholders::_1, std::placeholders::_2));
        lane_sub.AddReceiveCallback(std::bind(&motion_planinng::Update_Lane, this,std::placeholders::_1, std::placeholders::_2));
        free_sub.AddReceiveCallback(std::bind(&motion_planinng::UpdateFree, this,std::placeholders::_1, std::placeholders::_2));
        local_sub.AddReceiveCallback(std::bind(&motion_planinng::Updatelocalize, this,std::placeholders::_1, std::placeholders::_2));
        read_points();

    }
    std::vector<cv::Point> generate_wapoints(double lat, double lon, std::vector<std::vector<std::vector<cv::Point>>> free_homos, std::vector<std::vector<cv::Point>> lane_l, std::vector<std::vector<cv::Point>> lane_r)
    {
        double dis_lat = 10000;
        double dis_lon = 10000;
        std::vector<cv::Point> waypoints;
        int instruct = 0;
        int prev = 0;

        // Deciding how to generate waypoints

        // Getting the nearest lat long
        for (int i = 0; i < global_waypoints.size(); i++)
        {

            if (sqrt(pow((lat - global_waypoints[i].lat), 2) + pow((lon - global_waypoints[i].lon), 2)) < dis_lat)
            {

                dis_lat = sqrt(pow((lat - global_waypoints[i].lat), 2) + pow((lon - global_waypoints[i].lon), 2));

                instruct = i;
            }
        }

        if (lane_l[0].size() > 40 && lane_r[0].size() > 40)
        {
            for (int i = 0; i < 40; i++)
            {
                int mid_x = ((lane_l[0][i].x - lane_r[0][i].x) / 2) + 640 / 2 + 180 / 2;
                cv::Point way(mid_x, lane_l[0][i].y);
                waypoints.push_back(way);
            }
        }

        float factor = 10;
        float x_1 = 0;
        float x_2 = (global_waypoints[instruct + 10].lon - lon);
        float x_3 = (global_waypoints[instruct + 20].lon - lon);
        float y_1 = 0;
        float y_2 = (global_waypoints[instruct + 10].lat - lat);
        float y_3 = (global_waypoints[instruct + 20].lat - lat);

        std::vector<cv::Point> lane_l_free;
        std::vector<cv::Point> lane_r_free;


        // TODO - access confidence threshold from global variable
        if (conf_L < 0.1 || conf_L < 0.1)
        {
            for (auto frees_homo : free_homos)
            {
                for (auto free_homo : frees_homo)
                {
                    int i_r = 0;
                    int i_l = 0;
                    if (i_l > 120 && i_r > 120)
                    {
                        break;
                    }
                    for (auto point : free_homo)
                    {

                        if (point.x > 640)
                        {
                            if (point.y > i_r)
                            {
                                lane_l_free.push_back(point);
                                i_r = i_r + 30;
                            }
                        }
                        if (point.x < 640)
                        {
                            if (point.y > i_l)
                            {
                                lane_r_free.push_back(point);
                                i_l = i_l + 30;
                            }
                        }
                    }
                }
            }
            for (int i = 0; i < lane_l_free.size(); i++)
            {
                int mid_x = ((lane_l_free[i].x - lane_r_free[i].x) / 2) + 640 / 2 + 180 / 2;
                cv::Point way(mid_x, lane_l_free[i].y);
                waypoints.push_back(way);
            }
        }

        // Knowing which side to turn at T or L

        int flag; // left or right
        std::cout << "\n inst :" << instruct << std::endl;
        std::cout << "\n global :" << global_waypoints[instruct].angle << std::endl;
        if (global_waypoints[instruct].angle < ((3.14) / 4) || global_waypoints[instruct].angle > ((7 * 3.14) / 4))
        {
            trigger = 1;
        }
        else if (global_waypoints[instruct].angle > ((3.14) / 4))
            flag = 0;
        else if (global_waypoints[instruct].angle < ((7 * 3.14) / 4))
            flag = 1;
        cv::Point end_point(0, 640);
        for (auto frees_homo : free_homos)
        {
            for (auto free_homo : frees_homo)
            {
                for (auto point : free_homo)
                {
                    if (flag == 0)
                    {
                        if (point.x < 480) // change with lateral shift
                        {
                            if (end_point.y > point.y)
                            {
                                end_point.x = point.x;
                                end_point.y = point.y;
                            }
                        }
                    }
                    if (flag == 1)
                    {
                        if (point.x > 480) // change with lateral shift
                        {
                            if (end_point.y > point.y)
                            {
                                end_point.x = point.x;
                                end_point.y = point.y;
                            }
                        }
                    }
                }
            }
        }

        // TODO access map dimensions from global veriable

        std::vector<float> wx = {640 + x_1, 640 + x_2, 640 + x_3};
        std::vector<float> wy = {480 - y_1, 480 - y_2, 480 - y_3};
        std::vector<float> wx_1 = {end_point.x, 640};
        std::vector<float> wy_1 = {end_point.y, 480};

        robotics::Spline2D csp_obj(wx, wy);
        robotics::Spline2D csp_obj_2(wx_1, wy_1);

        int laneWidth = 20;
        if (flag == 1) // left = 1
            laneWidth = laneWidth * (-1);
        if (flag == 0) // right = 0
            laneWidth = laneWidth;
        waypoints.clear();

        // Intuitive Estimation of Turn

        for (float i = 0; i < csp_obj.s.back(); i += 0.1)
        {
            std::array<float, 2> point_ = csp_obj.calc_postion(i);
            if (i < csp_obj_2.s.back())
            {
                std::array<float, 2> point_1 = csp_obj_2.calc_postion(i);
                cv::Point tmp((int)(point_[0] + point_1[0]), (int)(point_[1] + point_1[1]));
                waypoints.push_back(tmp);
            }
        }

        // Roadside parking or stopping 

        if (instruct == global_waypoints.size() - 1)
        {
            int factor = 10;
            if (end_point.x * factor == 10)
            {
                cv::Point tmp(640, 480); // locallized point
                waypoints.push_back(tmp);
                tmp.x = lane_l[0][0].x + 10;
                tmp.y = end_point.y;
                waypoints.push_back(tmp);
                tmp.x = lane_l[0][0].x + 10;
                tmp.y = end_point.y - 10;

                waypoints.push_back(tmp);
                tmp.x = lane_l[0][0].x + 10;
                tmp.y = end_point.y - 40;
                waypoints.push_back(tmp);
            }
        }

        return waypoints;
    }

    std::vector<std::vector<cv::Point>> out_lane_l;
    
    // Update the to-go lane

    void Update_Lane(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
    {
        char* buffer=static_cast<char*>(data_->buf);
        std::stringstream ss(buffer);
        Serialize<Lanes> IPC;
        Lanes data;
        data = IPC.deserialize(ss,data);


        out_lane_l.clear();
        std::vector<cv::Point> tmp_l;
        for (int i = 0; i < data.lanesperframe_out[0][0].first.size(); i++)
        {
            cv::Point tmp(data.lanesperframe_out[0][0].first[i].x, data.lanesperframe_out[0][0].first[i].y);
            tmp_l.push_back(tmp);
        }
        conf_L=data.lanesperframe_out[0][0].second;
        conf_R=data.lanesperframe_out[0][1].second;
        MAX_ROAD_WIDTH_L = 640 - tmp_l[0].x;

        out_lane_l.push_back(tmp_l);
        out_lane_r.clear();
    
        std::vector<cv::Point> tmp_r;

        for (int i = 0; i < data.lanesperframe_out[0][1].first.size(); i++)
        {
            cv::Point tmp(data.lanesperframe_out[0][1].first[i].x, data.lanesperframe_out[0][1].first[i].y);
            tmp_r.push_back(tmp);
        }
        
        MAX_ROAD_WIDTH_R = tmp_r[0].x - 640;
        out_lane_r.push_back(tmp_r);
        std::vector<cv::Point> way;
        

        way = generate_wapoints(lat, lon, freespaces_homo, out_lane_l, out_lane_r);
        wx.clear();
        wy.clear();
        for (auto pts : way)
        {
            wx.push_back(pts.x);
            wy.push_back(pts.y);
        }

        onProcess();
        
    }
    std::vector<std::vector<cv::Point>> out_lane_r;
    float conf_R;
    float conf_L;
    
    std::vector<std::vector<std::vector<cv::Point>>> freespaces_homo;

    // TODO naming proper

    // Recieve freespace points from mapping
    void UpdateFree(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
    {
        char* buffer=static_cast<char*>(data_->buf);
        std::stringstream ss(buffer);
        Serialize<Freespace> IPC;
        Freespace data;
        data = IPC.deserialize(ss,data);
        freespaces_homo.clear();
        //(this->get_logger(), "in on free");
        std::vector<std::vector<cv::Point>> free_tmp;
        for (int i = 0; i < data.freespaces_homo[0].size(); i++)
        {
            std::vector<cv::Point> tmp_con;
            for (int j = 0; j < data.freespaces_homo[0][i].size(); j++)
            {
                cv::Point tmp(data.freespaces_homo[0][i][j].x, data.freespaces_homo[0][i][j].y);
                tmp_con.push_back(tmp);
            }
            free_tmp.push_back(tmp_con);
        }
        freespaces_homo.push_back(free_tmp);
    }
    float lat;
    float lon;
    
    // TODO Recieve localization from localization

    void Updatelocalize(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
    {
        char* buffer=static_cast<char*>(data_->buf);
        
        std::stringstream ss(buffer);
        Serialize<Odometry> IPC;
        Odometry data;
        data = IPC.deserialize(ss,data);
        
        lat = data.pose.pose.position.x;
        lon = data.pose.pose.position.y;
        c_speed = data.twist.twist.linear.x;
    }


    // TODO naming

    // Recieve obstacles from mapping

    void Update_weight(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
    {
        obstcles.clear();
        char* buffer=static_cast<char*>(data_->buf);
        
        std::stringstream ss(buffer);
        Serialize<Obstcales> IPC;
        Obstcales data;
        data = IPC.deserialize(ss,data);

        std::vector<int> labels;
        for (int i = 0; i < data.Obs.size(); i++)
        {
            Poi_f tmp_0 = {{data.Obs[i][0].x, data.Obs[i][0].y}};
            Poi_f tmp_1 = {{data.Obs[i][1].x, data.Obs[i][1].y}};
            Poi_f tmp_2 = {{data.Obs[i][2].x, data.Obs[i][2].y}};
            obstcles.push_back(tmp_0);
            obstcles.push_back(tmp_1);
            obstcles.push_back(tmp_2);
        }
    }

    // sum of power function

    float sum_of_power(std::vector<float> value_list)
    {
        float sum = 0;
        for (float item : value_list)
        {
            sum += item * item;
        }
        return sum;
    }

    // calculate frenet path

    Vec_Path calc_frenet_paths(
        float c_speed, float c_d, float c_d_d, float c_d_dd, float s0, Spline2D csp)
    {
        std::vector<FrenetPath> fp_list;
        for (float di = -1 * MAX_ROAD_WIDTH_L; di < MAX_ROAD_WIDTH_R; di += D_ROAD_W)
        {
            for (float Ti = MINT; Ti < MAXT; Ti += DT)
            {
                FrenetPath fp;
                QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
                for (float t = 0; t < Ti; t += DT)
                {
                    fp.t.push_back(t);
                    fp.d.push_back(lat_qp.calc_point(t));
                    fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                    fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
                    fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
                }
                for (float tv = TARGET_SPEED - D_T_S * N_S_SAMPLE;
                     tv < TARGET_SPEED + D_T_S * N_S_SAMPLE;
                     tv += D_T_S)
                {

                    FrenetPath fp_bot = fp;
                    QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

                    fp_bot.max_speed = std::numeric_limits<float>::min();
                    fp_bot.max_accel = std::numeric_limits<float>::min();
                    for (float t_ : fp.t)
                    {
                        fp_bot.s.push_back(lon_qp.calc_point(t_));
                    }
                    for (float t_ : fp.t)
                    {
                        fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
                    }
                    for (float t_ : fp.t)
                    {
                        fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
                    }
                    for (float t_ : fp.t)
                    {
                        fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
                        if (fp_bot.s_d.back() > fp_bot.max_speed)
                        {
                            fp_bot.max_speed = fp_bot.s_d.back();
                        }
                        if (fp_bot.s_dd.back() > fp_bot.max_accel)
                        {
                            fp_bot.max_accel = fp_bot.s_dd.back();
                        }
                    }

                    float Jp = sum_of_power(fp.d_ddd);
                    float Js = sum_of_power(fp_bot.s_ddd);
                    float ds = (TARGET_SPEED - fp_bot.s_d.back());

                    fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
                    fp_bot.cv = KJ * Js + KT * Ti + KD * ds;
                    fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv;

                    if (fp_bot.cf < 15)
                    {
                        for (unsigned int i = 0; i < fp_bot.s.size(); i++)
                        {
                            if (fp_bot.s[i] >= csp.s.back())
                            {
                                break;
                            }
                            std::array<float, 2> poi = csp.calc_postion(fp_bot.s[i]);
                            float iyaw = csp.calc_yaw(fp_bot.s[i]);
                            float di = fp_bot.d[i];
                            float x = poi[0] + di * std::cos(iyaw + M_PI / 2.0);
                            float y = poi[1] + di * std::sin(iyaw + M_PI / 2.0);
                            fp_bot.x.push_back(x);
                            fp_bot.y.push_back(y);
                        }

                        for (int i = 0; i < fp_bot.x.size() - 1; i++)
                        {
                            float dx = fp_bot.x[i + 1] - fp_bot.x[i];
                            float dy = fp_bot.y[i + 1] - fp_bot.y[i];
                            fp_bot.yaw.push_back(std::atan2(dy, dx));
                            fp_bot.ds.push_back(std::sqrt(dx * dx + dy * dy));
                        }

                        fp_bot.yaw.push_back(fp_bot.yaw.back());
                        fp_bot.ds.push_back(fp_bot.ds.back());

                        fp_bot.max_curvature = std::numeric_limits<float>::min();
                        for (int i = 0; i < fp_bot.x.size() - 1; i++)
                        {
                            fp_bot.c.push_back((fp_bot.yaw[i + 1] - fp_bot.yaw[i]) / fp_bot.ds[i]);
                            if (fp_bot.c.back() > fp_bot.max_curvature)
                            {
                                fp_bot.max_curvature = fp_bot.c.back();
                            }
                        }
                        fp_list.push_back(fp_bot);
                    }
                }
            }
        }
        return fp_list;
    };


    bool check_collision(FrenetPath path, const Vec_Poi ob)
    {
        for (auto point : ob)
        {
            for (unsigned int i = 0; i < path.x.size(); i++)
            {
                float dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
                if (dist <= ROBOT_RADIUS * ROBOT_RADIUS)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // check path

    Vec_Path check_paths(Vec_Path path_list, const Vec_Poi ob)
    {
        Vec_Path output_fp_list;
        for (FrenetPath path : path_list)
        {
            if (path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL && path.max_curvature < MAX_CURVATURE && check_collision(path, ob))
            {
                output_fp_list.push_back(path);
            }
        }
        return output_fp_list;
    }

    // frenet optimal planning

    FrenetPath frenet_optimal_planning(Spline2D csp, float s0, float c_speed, float c_d, float c_d_d, float c_d_dd, Vec_Poi ob)
    {
        Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, csp);

        Vec_Path save_paths = check_paths(fp_list, ob);

        float min_cost = std::numeric_limits<float>::max();
        // FrenetPath final_path;
        for (auto path : save_paths)
        {
            if (min_cost >= path.cf)
            {
                min_cost = path.cf;
                final_path = path;
            }
        }
        return final_path;
    }

    // For Rendering - TO_BE_DEPRECATED

    cv::Point2i cv_offset(float x, float y, int image_width = 2000, int image_height = 2000)
    {
        cv::Point2i output;
        output.x = int(x * 6) + 200;
        output.y = image_height - int(y * 6) - image_height / 3;
        image_width = 2000 + 0;
        return output;
    }



    void onProcess()
    {

        Vec_f w_x;
        Vec_f w_y;
        for (int i = 0; i < wx.size(); i++)
        {
            if (i % 50 == 0)
            {
                w_x.push_back(wx[i]);
                w_y.push_back(wy[i]);
            }
        }

        Spline2D csp_obj(w_x, w_y);

        r_x.clear();
        r_y.clear();
        rcurvature.clear();

        for (float i = 0; i < csp_obj.s.back(); i += 0.1)
        {
            std::array<float, 2> point_ = csp_obj.calc_postion(i);
            r_x.push_back(point_[0]);
            r_y.push_back(point_[1]);

            ryaw.push_back(csp_obj.calc_yaw(i));
            rcurvature.push_back(csp_obj.calc_curvature(i));
            rs.push_back(i);
        }

        FrenetPath final_path = frenet_optimal_planning(csp_obj, s0, c_speed, c_d, c_d_d, c_d_dd, obstcles);
        s0 = 0;
        c_d = final_path.d[1];
        c_d_d = final_path.d_d[1];
        c_d_dd = final_path.d_dd[1];
        path path_out;
        for(int i =0 ; i<final_path.x.size();i++)
        {
            path_out.x.push_back(final_path.x[i]);
            path_out.y.push_back(final_path.y[i]);
            path_out.v.push_back(final_path.s_d[i]); 
        }
        // TODO
        Serialize<path> data;
        std::stringstream ss;
        data.serialize(path_out,ss);
        std::string temp = ss.str();
        path_pub.Send(temp.c_str(), sizeof(temp));

        if (std::pow((final_path.x[1] - r_x.back()), 2) + std::pow((final_path.y[1] - r_y.back()), 2) <= 1.0)
        {
            exit(0);
        }

        double sum = 0;
        int flag = 0;
        for (uint64_t i = 0; i < obstcles.size(); i++)
        {
            sum = sum + sqrt(pow(final_path.x[0] - obstcles[i][0], 2.0) + pow((final_path.y[0] - obstcles[i][1]), 2));
            if (sqrt(pow(final_path.x[0] - obstcles[i][0], 2.0) + pow((final_path.y[0] - obstcles[i][1]), 2)) > 10)
                flag = 1;
        }
        double density = sum / obstcles.size();

        Behavior_planner(current_state, flag, density);

        cout << "speed" << TARGET_SPEED << "\n";

    
    }

    // TO_BE_DEPRECATED

    // void onRender()
    // {
    //     cv::Mat bg(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));

    //     for (unsigned int i = 1; i < r_x.size(); i++)
    //     {
    //         cv::Point tmp1(r_x[i - 1] * Scale, r_y[i - 1] * Scale);
    //         cv::Point tmp2(r_x[i] * Scale, r_y[i] * Scale);
    //         cv::line(
    //             bg,
    //             tmp1,
    //             tmp1,
    //             cv::Scalar(0, 0, 0),
    //             4);
    //     }
    //     const cv::Point *pts_1 = (const cv::Point *)cv::Mat(lane1).data;
    //     int npts_1 = cv::Mat(lane1).rows;

    //     // draw the polygon
    //     cv::polylines(bg, &pts_1, &npts_1, 1, false, cv::Scalar(0, 255, 0));
    //     /// lane 2
    //     const cv::Point *pts_2 = (const cv::Point *)cv::Mat(lane2).data;
    //     int npts_2 = cv::Mat(lane2).rows;

    //     cv::polylines(bg, &pts_2, &npts_2, 1, false, cv::Scalar(0, 255, 0));

    //     for (unsigned int i = 0; i < final_path.x.size(); i++)
    //     {

    //         cv::Point center(final_path.x[i] * Scale, final_path.y[i] * Scale);
    //         cv::circle(
    //             bg,
    //             center,
    //             4, cv::Scalar(255, 0, 0), -1);
    //     }
    //     cv::Point center(final_path.x.front() * Scale, final_path.y.front() * Scale);
    //     cv::circle(bg, center, 4, cv::Scalar(0, 255, 0), -1);

    //     for (unsigned int i = 0; i < obstcles.size(); i++)
    //     {
    //         cv::Point center(obstcles[i][0] * Scale - 7, obstcles[i][1] * Scale - 20);
    //         cv::Point center_2(obstcles[i][0] * Scale + 7, obstcles[i][1] * Scale);
    //         cv::rectangle(bg, center, center_2, cv::Scalar(0, 0, 255), 4, cv::LINE_8);
    //     }
    //     // cv::putText(bg,"Speed: " + std::to_string(c_speed*3.6).substr(0, 4) + "km/h",cv::Point2i((int)bg.cols*0.5, (int)bg.rows*0.1),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 0),5);
    //     cv::putText(bg, "acc: " + std::to_string(c_d_d * 3.6).substr(0, 4) + "km/h", cv::Point2i((int)bg.cols * 0.5, (int)bg.rows * 0.1), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 5);
    //     // cv::resize(bg, bg2, bg2.size(), 0, 0, cv::INTER_LINEAR);
    //     cv::imshow("frenet", bg);
    //     cv::waitKey(1);
    //     // cout<<"var in render = \n"<<var;;
    //     // var--;
    // }
};

int main(int argc, char *argv[])
{
    motion_planinng plan;

    eCAL::Initialize(argc, argv, "planning");
   
    
    // Just don't exit
    while (eCAL::Ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // finalize eCAL API
    eCAL::Finalize();  
    return 0;
}