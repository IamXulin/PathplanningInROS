#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <random>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
using namespace std;
#ifndef RRTSTAR_H
#define RRTSTAR_H

const int map_w=60;
const int map_h=60;

struct OBS_rec
{
    double x;
    double y;
    double len;
    double h;
};

struct OBS_circle
{
    double x;
    double y;
    double radius;
    OBS_circle(int x_value, int y_value, int r_value): x(x_value), y(y_value), radius(r_value) {}
};


struct Node
{
    double x;
    double y;
    double parent;
    double id;
    double cost;
    Node(double x_val, double y_val) : x(x_val), y(y_val), parent(-1), id(-1), cost(100000) {}
    Node() : x(0.0), y(0.0), parent(-1), id(-1), cost(100000) {}  // 默认构造函数
};

class RRT{
public:
    RRT(ros::NodeHandle &nh);
    void generate_obs();
    void visualization_obs();
    bool is_inside_obs(Node &node);
    bool is_intersect_circle(vector<double> org, vector<double> direc, 
                             vector<double> a, double r);
    bool is_collision(Node &start, Node &end);
    bool is_intersect_rec(Node &start, Node &end, vector<double> &org,
                          vector<double> &direc, vector<double> &a, vector<double> &b);
    vector<vector<vector<double>>> get_obs_vertex();
    Node generate_random_node();
    Node new_state(Node &node_start, Node &node_end , int id);
    vector<double> get_distance_and_angle(Node &node_start, Node &node_end);
    void extract_path(Node &node_end);
    Node nearest_neighbor(Node &node_rand);
    void planning();
    void visualization_vertex(Node &start, Node &end, int id);
    void init_start(visualization_msgs::Marker &pose);
    void init_goal(visualization_msgs::Marker &pose);
    void visualization_path();
    vector<int> find_near_neighbor(Node &node_new);
    double get_new_cost(Node &node_start, Node &node_end);
    void rewise(Node &node_new, vector<int> &neighbor_index ,int &id);
    void revisualization_vertex(Node &start, Node &end, int id);
    Node choose_parent(Node &node_new, vector<int> &neighbor_index);
    int search_goal_parent();
    
private:
    map<double,Node> Node_gather;
    ros::Publisher pubarry;
    ros::Publisher pubmark;
    vector<vector<double>> path;
    vector<Node> vertex;
    Node start=Node(1,1);
    Node goal=Node(48,28);
    double goal_sample_rate=0.05;
    double step_len=0.5;
    int iter_max=10000;
    double delta=0.5;
    double search_radius=20;
    visualization_msgs::MarkerArray Obs_gather;
    vector<OBS_circle> obs_circle;
    vector<OBS_rec> obs_rectangle;
    vector<OBS_rec> obs_boundary;
    vector<double> x_range={0,50};
    vector<double> y_range={0,30};

};


vector<vector<double>> get_ray(Node &start, Node &end);
double get_dist(Node &start, Node &end);

#endif
