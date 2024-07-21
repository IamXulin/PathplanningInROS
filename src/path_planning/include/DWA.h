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
#ifndef DWA_H
#define DWA_H
const int map_h = 30;
const int map_w = 30;

struct State
{
    double x;
    double y;
    double yaw;
    double V;
    double W;
};
struct Control
{
    double v;
    double w;
};
struct Obs
{
    int x;
    int y;
};



class DWA{
public:
    DWA(ros::NodeHandle &nh);
    
    vector<Obs> Obstacle;
    State ego_state;
    Control ego_control;
    visualization_msgs::Marker Path;
    visualization_msgs::Marker ego;
    ros::Publisher publi;

    void pub_obs();
    void Advertise_start(int init_x,int init_y);
    void Advertise_goal(int end_x,int end_y);
    void full_MarkerArray(int x,int y,int id,double scale_x,double scale_y,
                        double scale_z,double color_r,double color_g,
                        double color_b,double color_a,string choice);
    void dwa_control(vector<int> goal);
    void plot_path(vector<State> trajectory);
    void state_update();
    void trajectory_evaluation(vector<int> goal);
    vector<double> cal_dynamic_window_vel(double v,double w);
    vector<double> cal_vel_limit();
    vector<double> cal_accel_limit(double v, double w);
    vector<double> cal_obstacle_limit();
    vector<State> trajectory_evaluation();
    double obs_mindis();
    double eva_heading(vector<State> trajectory);
    double eva_dist(vector<State> trajectory);
    double eva_velocity(vector<State> trajectory);
    void initial_obs();

private:
    ros::Publisher pub;
    ros::Publisher pub_SG;
    visualization_msgs::MarkerArray Obs_gather;
    visualization_msgs::Marker start;
    visualization_msgs::Marker goal;
    
    
    vector<vector<int>> my_map=vector<vector<int>>(map_h,vector<int>(map_w,0));
    double dt=0.1;
    double v_min=-0.5;
    double w_min=-40.0*M_PI/180;
    double v_max=1.0;
    double w_max=40.0*M_PI/180;
    double predict_time=3.0;
    double a_vmax=0.2;
    double a_wmax=40.0*M_PI/180;
    double v_sample=0.01;
    double w_sample=0.1*M_PI/180;
    double alpha=0.15;
    double beta=1.0;
    double gamma=1.0;
    double radius=1.0;
    double judge_distance=2.0;
    int start_x;
    int start_y;
    int goal_x;
    int goal_y;


};


vector<State> trajectory_predict(State state_init,double v, double w, double predict_time);
State KinematicModel(State state,double v, double w,double dt);


#endif