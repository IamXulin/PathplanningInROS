#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <Eigen/Dense>
using namespace std;

#ifndef DUBINS_H
#define DUBINS_H





struct PATH
{
    double L;
    string mode;
    vector<double> x_list;
    vector<double> y_list;
    vector<double> yaw_list;
    PATH(double L1, string mo, vector<double> x1, vector<double> y1, vector<double> yaw1): 
                            L(L1),mode(mo),x_list(x1),y_list(y1),yaw_list(yaw1) {}
};
struct Trajectory
{
    double t=std::numeric_limits<double>::quiet_NaN(); 
    double p=std::numeric_limits<double>::quiet_NaN(); 
    double q=std::numeric_limits<double>::quiet_NaN();
    string mode;
};

struct Best_trajectory
{
    vector<double> x_list;
    vector<double> y_list;
    vector<double> yaw_list;
    string best_mode;
    double best_cost;
};

struct Dubins_Path
{
    vector<double> x;
    vector<double> y;
    vector<double> yaw;
};



double pi_2_pi(double theta);
double mod2pi(double theta);
Trajectory LSL(double alpha, double beta, double dist);
Trajectory RSR(double alpha, double beta, double dist);
Trajectory LSR(double alpha, double beta, double dist);
Trajectory RSL(double alpha, double beta, double dist);
Trajectory RLR(double alpha, double beta, double dist);
Trajectory LRL(double alpha, double beta, double dist);
Best_trajectory planning_from_origin(double gx, double gy, double gyaw,
                                     double curv, double step_size);
Best_trajectory generate_local_course(double L, vector<double> lengths,
                string mo, double maxc, double step_size);
void interpolate(int ind,double l,char m,double maxc,double ox,
                    double oy,double oyaw,vector<double> &px,vector<double> &py,
                    vector<double> &pyaw);
PATH calc_dubins_path(double sx, double sy, double syaw, double gx, double gy, double gyaw,
                        double curv, double step_size=0.1);


double deg2rad(double degrees);

Dubins_Path generate_path(vector<vector<double>> &states);

void visualization_path(Dubins_Path &path, ros::Publisher &pub_marker);
#endif