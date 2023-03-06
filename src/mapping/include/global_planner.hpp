#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <math.h>
#include <vector>
#include <iostream>
#include "cubic_spline.hpp"
#include <iterator>
struct instructions
{
    float lat;
    float lon;
    float angle;

};
using namespace std;
std::map<int,instructions>  get_global_waypoints(std::vector<std::vector<double>> pos_to_track)
{
   
    std::map<int,instructions> global_waypoints;
    int i=0;
    for(;i<(pos_to_track.size()-20);i++)
    {
        

        float ab_x,ab_y,bc_x,bc_y;
        ab_x = pos_to_track[i+10][0] - pos_to_track[i][0];  //angle between three points https://stackoverflow.com/questions/3486172/angle-between-3-points
        ab_y = pos_to_track[i+10][1] - pos_to_track[i][1];
        bc_x = pos_to_track[i+10][0] - pos_to_track[i+20][0];  //angle between three points https://stackoverflow.com/questions/3486172/angle-between-3-points
        bc_y = pos_to_track[i+10][1] - pos_to_track[i+20][1];
    
        float angba = atan2(ab_y, ab_x);
        float angbc = atan2(bc_y, bc_x);
        float rslt = angba - angbc;
        instructions a{pos_to_track[i][0],pos_to_track[i][1],rslt+3.1415};
        global_waypoints.insert(std::pair<int, instructions>(i, a));
        
    }
    instructions a{pos_to_track[pos_to_track.size()-2][0],pos_to_track[pos_to_track.size()-2][1],0};
    global_waypoints.insert(std::pair<int, instructions>(i, a));
    instructions a_1{pos_to_track[pos_to_track.size()-1][0],pos_to_track[pos_to_track.size()-1][1],0};
    global_waypoints.insert(std::pair<int, instructions>(i+1, a_1));
    
    return global_waypoints;
    
    

}
