#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include"ros/ros.h"
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
using namespace std;
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

const int map_h = 30;
const int map_w = 30;
class DIJKSTRA{
public:
    ros::Subscriber sub_start;
    ros::Subscriber sub_goal;
    ros::Publisher pub;
    ros::Publisher pub_SG;
    visualization_msgs::Marker PlotPath;
    visualization_msgs::MarkerArray Process;
    vector<vector<int>> my_map=vector<vector<int>>(map_h,vector<int>(map_w,0));
    void full_PathArray(int x,int y);
    void planning();
    void full_MarkerArray(int x,int y,int id,double scale_x,double scale_y,
                        double scale_z,double color_r,double color_g,
                        double color_b,double color_a,string choice);
    void generate_obs();
    void pub_obs();
    void dostart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);
    void dogoal(const geometry_msgs::PoseStamped::ConstPtr &pose);
    bool Iscollision(int next_x,int next_y);
    void Advertise_process();
    void Advertise_start(int init_x,int init_y);
    void Advertise_goal(int end_x,int end_y);
    void full_path();
private:
    int start_x,start_y,goal_x,goal_y;
    vector<vector<int>> motions={{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};
    //vector<vector<int>> motions={{1,0},{0,1},{-1,0},{0,-1}};
    vector<vector<int>> path;
    map<int,vector<int>> Nodelist;
    map<vector<int>,int> Nodelistreverse;
    map<vector<int>,int> Parentlist;
    map<int,double> g_cost;
    vector<vector<int>> openlist,closedlist;
    
    visualization_msgs::MarkerArray Obs_gather;
    
    
    visualization_msgs::Marker start;
    visualization_msgs::Marker goal;
    
    

};

#endif