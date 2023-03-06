// #include "global_planner.hpp"
#include <vector>
#include <math.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <sstream>
#include <map>
#include <iostream>
#include <thread>

// IPC
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "global_planner.hpp"
#include "centroidtracker.hpp"
#include "cubic_spline.hpp"
#include "commons.hpp"


#include "perception/perceptionDS.hpp"

#include "mapping/mappingDS.hpp"

float F_LANEDEVIATION = 1;
auto centroidTracker = new CentroidTracker(10);

using std::placeholders::_1;

class mapping
{
private:
    std::vector<std::vector<int>> v_boxes;

    map<int, const char> m_labelName;
    int a_lableList[11] = {0, 1, 2, 3, 5, 7, 10, 15, 16, 18, 19};

    int iter = 0;
    std::vector<double> v_angle;
    
    // generate point gap acc to classes

    std::vector<cv::Point> gen_pts(int class_id, cv::Point s)
    {
        std::vector<cv::Point> pts_1;

        std::vector<cv::Point> tilted_pts;

        int a;
        if (class_id == 0)
            a = 20;
        else if (class_id == 1)
            a = 30;
        else if (class_id == 2)
            a = 15;
        else if (class_id == 3)
            a = 20;
        else if (class_id == 4)
            a = 40;
        else if (class_id == 5)
            a = 10;
        else if (class_id == 6)
            a = 5;
        else if (class_id == 7)
            a = 5;
        else if (class_id == 8)
            a = 5;
        else if (class_id == 9)
            a = 10;
        else if (class_id == 12)
            a = 10;
        else if (class_id == 13)
            a = 40;
        else if (class_id == 15)
            a = 15;
        uint16_t k = 0;

        for (int i = 0; i < 3; i++)
        {
            cv::Point d(s.x, s.y - k);
            k = k + a;
            pts_1.push_back(d);
        }

        return add_tilt(class_id, pts_1);
    }

    // TODO spelling mistakes correct

    std::vector<std::vector<int>> boxes;
    std::vector<std::vector<std::vector<cv::Point>>> free_homo;
    std::vector<std::pair<std::vector<cv::Point>, float>> lanesperframe;
    std::vector<std::pair<std::vector<cv::Point>, float>> lane_l;
    std::vector<cv::Point> lane_l_stabalized;
    std::vector<cv::Point> lane_r_stabalized;
    std::vector<std::vector<cv::Point>> lane_l_history;
    std::vector<std::vector<cv::Point>> lane_r_history;
    std::vector<std::pair<std::vector<cv::Point>, float>> lane_r;
    std::vector<std::vector<cv::Point>> obs_generated;
    std::vector<cv::Point> obs;
    std::vector<cv::Point> lane_1_tmp;
    std::vector<cv::Point> lane_2_tmp;
    cv::Point end_point;
    std::vector<cv::Point> obs_tmp;
    std::vector<double> angle;
    uint64_t sz = 0;
    uint64_t pts_size = 0;

    // Adds tilt w.r.t to lane
    // TODO How are you going to do tilt if lanes are crazy AF?

    std::vector<cv::Point> add_tilt(int class_id, std::vector<cv::Point> s)
    {

        cv::Point top;
        cv::Point bottom;

        for (uint i = 0; i < pts_size; i++)
        {

            if (lane_l[0].first[i].y > s[2].y)
            {

                top = lane_l[0].first[i];
            }
            else
            {
                break;
            }

            if (lane_l[0].first[i].y > (s[0].y))
            {
                bottom = lane_l[0].first[i];
            }
        }

        int offset = (top.x - bottom.x) / 2;

        angle.push_back(-(std::atan2(20, offset) * (180 / 3.1415)));

        cv::Point tmp(s[2].x + offset, s[2].y);
        std::vector<cv::Point> pts;
        pts.push_back(tmp);
        cv::Point tmp2(s[0].x - offset, s[0].y);
        pts.push_back(s[1]);
        pts.push_back(tmp2);

        return pts;
    }
    
    cv::Point lateral_shift(cv::Point point)
    
    {
        int lane_width = 180;
        point.x = point.x + 640 / 2 - lane_width / 2;
        return point;
    }

    cv::Point lateral_shift1(cv::Point point)
    {
        int lane_width = 180;
        point.x = point.x + 640 / 2 - 100;
        return point;
    }

    //  TODO spelling mistake 
    
    cv::Point cordinte_negate_to_opencv(cv::Point point)
    {
        int lane_width = 180;
        point.y = -1 * point.y + 480;
        point.x = point.x + 640 / 2 - lane_width / 2;
        return point;
    }

    cv::Point rotate_transform(cv::Point point, float theta)
    {
        cv::Point origin(640 / 2, 0);
        cv::Point linear(640, 0);
        point.y = 480 - point.y;

        float r = cv::norm(origin - point);
        float angba = atan2(point.y - origin.y, point.x - origin.x);
        float angbc = atan2(linear.y - origin.y, linear.x - origin.x);
        float radian = angba - angbc;
        radian = radian + theta;
        cv::Point new_point(r * (cos(radian)), r * (sin(radian)));

        return new_point;
    }
    
    //  Homography
    cv::Point performHomography(cv::Point point, int x)
    {

        // TODO access homography matrix from global variables
        int32_t px, py;
        point.x = (((-1.39648437e-01) * point.x) + (-1.98166233e+00 * point.y) + 3.71929688e+02) / ((6.44339506e-19 * point.x) + (-6.07638889e-03 * point.y) + 1.00000000e+00);
        point.y = (((2.61990034e-16) * point.x) + (-2.90798611e+00 * point.y) + 5.36736111e+02) / ((6.44339506e-19 * point.x) + (-6.07638889e-03 * point.y) + 1.00000000e+00);

        return point;
    }

    std::vector<double> get_equtation(double x_1, double y_1, double x_2, double y_2)
    {
        double a = y_1 / (pow((x_1 - x_2), 2) + y_2);
        double b = 2 * x_2 * a;
        double c = (pow(x_2, 2) * a) + y_2;
        std::vector<double> res = {a, b, c};
        return res;
    }

public:
    
    float conf_R;
    float conf_L;
    std::map<int, instructions> global_waypoints;
    std::vector<std::vector<std::string>> contents;
    std::vector<cv::Point> waypoints;
    eCAL::CSubscriber obj_sub{"obj"};
    eCAL::CSubscriber lane_sub{"lanes"};
    eCAL::CSubscriber free_sub{"free"};
    eCAL::CPublisher lane_pub{"lanes_homo"};
    eCAL::CPublisher obs_pub{"obs_homo"};
    eCAL::CPublisher free_pub{"free_homo"};
    mapping()
    {
        obj_sub.AddReceiveCallback(std::bind(&mapping::UpdateObs, this,std::placeholders::_1, std::placeholders::_2));
        lane_sub.AddReceiveCallback(std::bind(&mapping::UpdateLanes, this,std::placeholders::_1, std::placeholders::_2));
        free_sub.AddReceiveCallback(std::bind(&mapping::UpdateFreespace, this,std::placeholders::_1, std::placeholders::_2));

    }
    void onProcessObtacles(std::vector<cv::Point> obs, std::vector<int> labels)
    {
        
        obs_generated.clear();
        iter = 0;
        for (uint64_t k = 0; k < obs.size(); k++)
        {
            if (labels[k] != 11 || labels[k] != 14)
            {
                std::vector<cv::Point> pts;
                pts = gen_pts(labels[k], lateral_shift(performHomography(obs[k], F_LANEDEVIATION)));
                obs_generated.push_back(pts);
            }
        }
        Obstcales obss;
        obss.Obs=obs_generated;
        obss.labels=labels;
        Serialize<Obstcales> data;
        std::stringstream ss;
        data.serialize(obss,ss);
        std::string temp = ss.str();
        obs_pub.Send(temp.c_str(), sizeof(temp));
              
    }

//   TODO - don't confuse poeple with ROS. Rename
//  IPC

    void UpdateObs(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
    {
        char* buffer=static_cast<char*>(data_->buf);

        // obs.clear();
   		std::stringstream ss(buffer);
        Serialize<ObjectDetectionData> IPC;
        ObjectDetectionData data;
        data = IPC.deserialize(ss,data);

        std::vector<int> labels;
        for (int i = 0; i < data.m_detectedCentroids.size(); i++)
        {
            for(int j = 0; j < data.m_detectedCentroids[i].size();j++)
            {
                cv::Point tmp(data.m_detectedCentroids[i][j].x, data.m_detectedCentroids[i][j].y);
                obs.push_back(tmp);
                labels.push_back(data.classIds[i][j]);
            }
        }
        onProcessObtacles(obs, labels);
       
    }
    
    
    std::vector<std::vector<std::pair<std::vector<cv::Point>, float>>> lanesperframe_out;

    void OnProcessLanes(std::vector<std::pair<std::vector<cv::Point>, float>> lanesperframe)
    {   
        
        // TEMPORAL SMOOTHENING AND OTHER THINGS

        std::vector<cv::Point> single_l_lane;
        std::vector<cv::Point> single_r_lane;
        if (lanesperframe.size() > 0)
        {
            for (uint32_t j = 0; j < lanesperframe[0].first.size(); j++)
            {
                if (lanesperframe[0].first[j].x > 0 && lanesperframe[0].first[j].y > 0 && lanesperframe[0].first[j].x < 640 && lanesperframe[0].first[j].y < 480)
                {
                    cv::Point tmp(lanesperframe[0].first[j].x, lanesperframe[0].first[j].y);
                    tmp = lateral_shift(performHomography(tmp, F_LANEDEVIATION));
                    
                    if (tmp.x > 0 && tmp.y > 0 && tmp.x < 640 && tmp.y < 480)
                        single_l_lane.push_back(tmp);
                }
            }
            std::pair<std::vector<cv::Point>,float> tmp_ = {single_l_lane,lanesperframe[0].second};
            lane_l.push_back(tmp_);
        }
        if (lanesperframe.size() > 1)
        {
            for (uint32_t j = 0; j < lanesperframe[1].first.size(); j++)
            {
                if (lanesperframe[1].first[j].x > 0 && lanesperframe[1].first[j].y > 0 && lanesperframe[1].first[j].x < 640 && lanesperframe[1].first[j].y < 480)
                {
                    cv::Point tmp2(lanesperframe[1].first[j].x, lanesperframe[1].first[j].y);
                    tmp2 = lateral_shift(performHomography(tmp2, F_LANEDEVIATION));
                    if (tmp2.x > 0 && tmp2.y > 0 && tmp2.x < 640 && tmp2.y < 480)
                        single_r_lane.push_back(tmp2);
                }
            }
            std::pair<std::vector<cv::Point>,float> tmp_ = {single_r_lane,lanesperframe[1].second};
            lane_l.push_back(tmp_);
        }
        lanesperframe_out.push_back(lane_l);
        lanesperframe_out.push_back(lane_r);
        Lanes lane_out;
        lane_out.lanesperframe_out = lanesperframe_out;
        Serialize<Lanes> data;
        std::stringstream ss;
        data.serialize(lane_out,ss);
        std::string temp = ss.str();
        lane_pub.Send(temp.c_str(), sizeof(temp));
        

        

//  Second Method of Temporal Smoothenin

        // lane_l_stabalized.clear();
        //     if(lane_l.size()>0)
        //         {
        //             lane_l_history.push_back(lane_l[0]);
        //             if (lane_l_history.size()>5)
        //             {
        //                 lane_l_history.erase(lane_l_history.begin());
        //                 for(int i = 0;i<lane_l_history[0].size();i++)
        //             {
        //                 int sum=lane_l_history[0][i].x;
        //                 int count=1;
        //                 for(int j = 1;j<5;j++)
        //                 {
        //                     if(lane_l_history[0][i].y-lane_l_history[j][i].y<10)
        //                         {
        //                             sum=sum+lane_l_history[j][i].x;
        //                             count++;
        //                         }

        //                 }
        //                 cv::Point tmp(sum/count,lane_l_history[0][i].y);
        //                 lane_l_stabalized.push_back(tmp);
        //             }
        //             }
        //         }
        // lane_r_stabalized.clear();
        //     if(lane_r.size()>0)
        //         {
        //             lane_r_history.push_back(lane_r[0]);
        //             if (lane_r_history.size()>5)
        //             {
        //                 lane_r_history.erase(lane_r_history.begin());
        //                 for(int i = 0;i<lane_r_history[0].size();i++)
        //             {
        //                 int sum=lane_r_history[0][i].x;
        //                 int count=1;
        //                 for(int j = 1;j<5;j++)
        //                 {
        //                     if(lane_r_history[0][i].y-lane_r_history[j][i].y<10)
        //                         {
        //                             sum=sum+lane_r_history[j][i].x;
        //                             count++;
        //                         }

        //                 }
        //                 cv::Point tmp(sum/count,lane_r_history[0][i].y);
        //                 lane_r_stabalized.push_back(tmp);
        //             }
        //             }
        //         }
        //     if(lane_l[0].size()>0 && lane_r[0].size()>0)
        //         F_LANEDEVIATION = (lane_l[0][lane_l[0].size()/2].x-lane_r[0][lane_l[0].size()/2].x)/(lane_l[0][0].x-lane_r[0][0].x);

        
    }

    // TODO Why again the same confusion????

    void UpdateLanes(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
    {
        lane_l.clear();
        lane_r.clear();
        char* buffer=static_cast<char*>(data_->buf);
        std::stringstream ss(buffer);
        Serialize<LaneEstimationData> IPC;
        LaneEstimationData data;
        data = IPC.deserialize(ss,data);
        std::vector<std::vector<std::pair<std::vector<cv::Point>, float>>> lanesConsloidated;
        std::vector<std::pair<std::vector<cv::Point>, float>> lanesperframe;

        std::vector<cv::Point> tmp_l;
        for (int i = 0; i < data.detectedLanes[0][0].first.size(); i++)
        {
            cv::Point tmp(data.detectedLanes[0][0].first[i].x, data.detectedLanes[0][0].first[i].y);
            tmp_l.push_back(tmp);
        }
        std::pair<std::vector<cv::Point>, float> a(tmp_l, data.detectedLanes[0][0].second);
        lanesperframe.push_back(a);
        tmp_l.clear();
        for (int i = 0; i < data.detectedLanes[0][1].first.size(); i++)
        {
            cv::Point tmp(data.detectedLanes[0][1].first[i].x, data.detectedLanes[0][1].first[i].y);
            tmp_l.push_back(tmp);
        }
        std::pair<std::vector<cv::Point>, float> b(tmp_l, data.detectedLanes[0][1].second);
        lanesperframe.push_back(b);
        OnProcessLanes(lanesperframe);
    }
   

    void onProcessFreespace(std::vector<std::vector<std::vector<cv::Point>>> freespaces_homo, std::vector<float> angle)
    {
        end_point.x = 640;
        end_point.y = 480;
        int k = 0;
        for (auto freespace_homo : freespaces_homo)
        {
            
            std::vector<std::vector<cv::Point>> contours;

            for (uint32_t j = 0; j < freespace_homo.size(); j++)
            {
                std::vector<cv::Point> single_contour;
                for (uint32_t i = 0; i < freespace_homo[j].size(); i++)
                {

                    cv::Point tmp2(freespace_homo[j][i].x, freespace_homo[j][i].y);
                    tmp2 = lateral_shift(cordinte_negate_to_opencv(rotate_transform(performHomography(tmp2, F_LANEDEVIATION), angle[k])));
                    single_contour.push_back(tmp2);
                }
                contours.push_back(single_contour);
            }
            free_homo.push_back(contours);
            k++;
        }
        Freespace free;
        free.freespaces_homo = free_homo;
        Serialize<Freespace> data;
        std::stringstream ss;
        data.serialize(free,ss);
        std::string temp = ss.str();
        free_pub.Send(temp.c_str(), sizeof(temp));

    }

    // TODO - Ohh my god! Nehalllll. I mean whyyyyyyy

    void UpdateFreespace(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_)
    {
        char* buffer=static_cast<char*>(data_->buf);
        int cam_id = 0;
        std::vector<float> angle;
        
        std::stringstream ss(buffer);
        Serialize<FreeSpaceData> IPC;
        FreeSpaceData data;
        data = IPC.deserialize(ss,data);
        std::vector<std::vector<std::vector<cv::Point>>> freespaces_homo;
        std::vector<std::vector<cv::Point>> free_tmp;
        for (auto freehomo:data.frameFreeSpacePoints)
        {
            for (int i = 0; i < freehomo.size(); i++)
            {
                std::vector<cv::Point> tmp_con;
                for (int j = 0; j < freehomo[i].size(); j++)
                {
                    cv::Point tmp(freehomo[i][j].x, freehomo[i][j].y);
                    tmp_con.push_back(tmp);
                }
                free_tmp.push_back(tmp_con);
            }
            if (free_tmp.size() > 0)
                freespaces_homo.push_back(free_tmp);
        }

        
        onProcessFreespace(freespaces_homo, angle);
       
    }


    int onRelease()
    {
       return -1;
    }

    int onReset()
    {
        return -1;
    }

};
int main(int argc, char *argv[])
{
    mapping mapObj;

    eCAL::Initialize(argc, argv, "mapping");
   
    
    // Just don't exit
    while (eCAL::Ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // finalize eCAL API
    eCAL::Finalize();
   
    return 0;
}
