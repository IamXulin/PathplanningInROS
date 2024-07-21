#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float64.h"
#include"ros/ros.h"
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
using namespace std;
#ifndef MPC_H
#define MPC_H




struct reference_Node
{
    double x;
    double y;
    double yaw;
    double karry;
    double v=3.0;
};
struct Position
{
    double x;
    double y;
    double yaw;
};




class MPC{

public:
    
    MPC(ros::Publisher pub1, ros::Publisher pub2);
    void curve_generate();
    void referencepath_generate();
    void init_ego();
    Position ego_position;
    vector<geometry_msgs::Point> point_gather;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker ego;
    int indexx=0;
    bool init_run=false;
    vector<reference_Node> reference_path;
    Eigen::MatrixXd Ad_;
    Eigen::MatrixXd Bd_;
    
    Eigen::VectorXd kesi;
    Eigen::MatrixXd Qq;
    Eigen::MatrixXd Rr;
    vector<double> uk_1{0.0,0.0};
    vector<double> cal_target_reference(double x,double y);
    void linerization(const double &phi_r, const double &v_r,const double &delta_r);
    void update(vector<double> &out);
    void solveQP();
    Eigen::MatrixXd qiumi(Eigen::MatrixXd A,int u);
    void pub_path();
private:
    ros::Publisher pub;
    ros::Publisher pub_err;
    const int Nx=3;
    const int Nu=2;
    double L=2.0;
    double dt=0.1;
    int Np=40;
    int Nc=20;
    double umin_v=-0.2;
    double umin_throll=-0.54;
    double umax_v=0.2;
    double umax_throll=0.332;
    double delta_umin_v=-0.05;
    double delta_umin_throll=-0.64;
    double delta_umax_v=0.05;
    double delta_umax_throll=0.64;
    double row=10.0;

};

#endif