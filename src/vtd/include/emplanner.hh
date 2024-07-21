#include "ros/ros.h"
#include <iostream>
#include "frenet_path.h"
#include "pid_controller.h"
#include "LQR_Control.h"
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Path.h"
#include "frenet_optimal_trajectory.h"
using namespace std;
using namespace nav_msgs;

template <typename U, typename V>
double DistanceXY(const U &u, const V &v) {
    return std::hypot(u.x - v.x, u.y - v.y);
}

class PathPlanningNode {
public:
    PathPlanningNode();
    ~PathPlanningNode();
    bool init();
private:
    void UpdateVehicle(const double acc, const double steer);
    void publishVehicle();
    void PlotGlobalPath();
    void plannerTimerLoop(const ros::TimerEvent &);
    void controlTimerLoop(const ros::TimerEvent &);
    void visTimerLoop(const ros::TimerEvent &);
    double pid_control(double vr);
    bool loadRoadmap(const double target_speed);
    void addRoadmapMarker(const std::vector<TrajectoryPoint> &path);
    void UpdateStaticObstacle();//添加障碍物
    void UpdateDynamicObs();//更新动态障碍物状态
    void publishObstacles();
    void odomCallback();
    void publishLocalPlan(const FrenetPath &final_path);

    //将规划的自然系路径转化为直角坐标系路径
    TrajectoryData GetTrajectoryFormFrenetPath(const FrenetPath &path);
    void GetWayPoints();//获取粗略参考点
    int GetNearestReferenceIndex(const VehicleState &ego_state);//获取车辆参考点索引
    double GetNearestReferenceLength(const VehicleState &ego_state);//获取车辆参考点s值
    double GetNearestReferenceLatDist(const VehicleState &ego_state);//获取车辆参考点l值
    bool LeftOfLine(const VehicleState &p, const geometry_msgs::PoseStamped &p1,
                  const geometry_msgs::PoseStamped &p2);//判断车辆在参考线哪一侧
    
    ros::NodeHandle nh_; 
    ros::Timer visTimer_;                            //可视化的线程
    ros::Timer controlTimer_;                        //控制的线程
    ros::Timer plannerTimer_;                        //规划的线程
    ros::Publisher pub_marker;
    double targetSpeed_ = 5;
    std::shared_ptr<PIDController> speedPidControllerPtr_;
    std::shared_ptr<LQRController> lqrController_;

    double controlFrequency_ = 100;  //控制频率
    double plannerFrequency_ = 10;   //规划频率
    VehicleState vehicleState_;

    TrajectoryData planningNodePublishedTrajectory_;
    TrajectoryData planningPublishedTrajectoryDebug_;  //规划下发的轨迹
    TrajectoryData last_trajectory_;                   //规划下发的轨迹
    TrajectoryData planningPublishedTrajectory_;       //跟踪的轨迹
    TrajectoryPoint goalPoint_;                        //终点
    double goalTolerance_ = 0.5;                       //到终点的容忍距离
    bool isReachGoal_ = false;
    bool firstRecord_ = true;
    bool plannerFlag_ = false;
    double target_speed=5.0;
    Vec_f wx_, wy_;

    Spline2D *csp_obj_;

    float c_speed_ = 10.0 / 3.6;
    float c_d_ = 2.0;
    float c_d_d_ = 0.0;
    float c_d_dd_ = 0.0;
    float s0_ = 0.0;
    FrenetPath last_final_path;
    nav_msgs::Path global_plan_;

    std::vector<Poi_f> obstcle_list_;//障碍物集合
    std::vector<VehicleState> dynamic_obstcle_list_;//动态障碍物集合
    double end_x_, end_y_, end_s_;
    bool near_goal_ = false;
    bool use_reference_line_ = false;














    //计算两点之间的距离
    double pointDistance(const TrajectoryPoint &point, const double x,
                        const double y) {
        double dx = point.x - x;
        double dy = point.y - y;
        return sqrt(dx * dx + dy * dy);
    }
    double pointDistance(const Poi_f &point, const double x, const double y) {
        double dx = point[0] - x;
        double dy = point[1] - y;
        return sqrt(dx * dx + dy * dy);
    }
    double pointDistance(const double x1, const double y1, const double x,
                        const double y) {
        double dx = x1 - x;
        double dy = y1 - y;
        return sqrt(dx * dx + dy * dy);
    }
};