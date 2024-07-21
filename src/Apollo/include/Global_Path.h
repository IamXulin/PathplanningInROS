#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include <vector>
#include <random>
#include <limits>
#include <cmath>
#include <Eigen/Dense>
#include <tuple>
#ifndef GLOBAL_H
#define GLOBAL_H
#define SMOOTH_COST 10
#define LENGTH_COST 2
#define REF_COST 3
#define BUFF_UB 2.0
#define BUFF_LB -2.0
#define L 3.0
using namespace std;
struct GlobalNode
{
    double x;
    double y;
    double heading;
    double kappa;

};

struct Host_State
{
    double x=0.0;
    double y=0.0;
    double heading_xy=0.0;
    double vx=0.0;
    double vy=0.0;
    double ax=0.0;
    double ay=0.0;
};




void Ego_State_Update(Host_State& ego , double dt, double steer);

std::tuple<vector<double>, vector<double>> UnSmoothPath(const int &match_index,
                    vector<double> global_x, vector<double> global_y);


int referenceline_provider_seek_match_index(const double &host_x, const double &host_y,
                const vector<double> &global_x, const vector<double> &global_y,
                const vector<double> &global_heading, const vector<double> &global_kappa);

std::tuple<vector<double>, vector<double>> Cal_heading_kappa(
                const vector<double> &reference_x,const vector<double> &reference_y);



std::tuple<vector<double>,vector<double>> generate_globalpath();


vector<GlobalNode> Get_SmoothReference(const vector<double> &reference_x,
                                        const vector<double> &reference_y);

std::tuple<vector<double>,vector<double>> Smooth_Globalpath(vector<double> reference_x,
                                vector<double> reference_y);





#endif //GLOBAL_H