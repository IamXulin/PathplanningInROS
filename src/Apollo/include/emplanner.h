#include "Lattice.h"
#include "SpeedPlanning.h"
#include "Path_QP.h"
#include "FrenetToCartesian.h"
#include <tf2/LinearMath/Quaternion.h>
#include "ros/ros.h"
#include "PID_Control.h"
#include "LQR_Control.h"
#include <thread>
#include <chrono>
#ifndef EMPLANNER_H
#define EMPLANNER_H
class Path_planning_node {
public:
    Path_planning_node();
    ~Path_planning_node();
    bool Init();
private:
    void UpdateObs();
    void EM_Planner(const ros::TimerEvent& event);
    void Control(const ros::TimerEvent& event);
    void VisualizaPath(const ros::TimerEvent& event);

    void graph_obs();
    void graph_local();
    void graph_host();
    void graph_global_path();


    ros::NodeHandle nh_; 
    ros::Timer visTimer_;                            //可视化的线程
    ros::Timer controlTimer_;                        //控制的线程
    ros::Timer plannerTimer_;                        //规划的线程
    ros::Publisher pub_marker;

    std::shared_ptr<PIDController> speedPidControllerPtr_;
    std::shared_ptr<PIDController> locationPidControllerPtr_;
    std::shared_ptr<LQRController> lqrController_;
    std::shared_ptr<FindHostOriginPoint> Host_Proj;
    std::shared_ptr<FindHostOriginPoint> Host_Origin;
    std::shared_ptr<FindHostOriginPoint> Plan_Start_Origin;
    Host_State host;
    double controlFrequency_ = 100;  //控制频率
    double plannerFrequency_ = 10;   //规划频率
    double vis_frequency_=50;
    Plan_Trajectory pre_trajectory;
    Plan_Trajectory Trajectory_final;
    
    std::shared_ptr<Static_Obs> Handle_Obs;
    std::shared_ptr<Lattice> Lattice_Planner;
    std::shared_ptr<PATH_QP_Solver> Path_QP_Planner;
    
    std::vector<double> global_x;
    std::vector<double> global_y;
    std::vector<double> global_heading;
    std::vector<double> global_kappa;

    vector<Obs_State> Obs_gather;

    bool pland=false;
    bool init_run=true;
    int start_index;
};

#endif