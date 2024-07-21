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
ros::Publisher pub;
bool planed=false;
visualization_msgs::Marker line_marker;
visualization_msgs::Marker Obs_marker;
visualization_msgs::Marker Host_marker;
visualization_msgs::Marker local_path;
std::atomic<bool> keepRunning(true);
PIDController PID_distance(0.5, 0.0, 0.0);
PIDController PID_speed(1.8, 0.0, 0.0);
LQRController LQR_Contrller;

vector<double> obs_x_set_gcs;
vector<double> obs_y_set_gcs;
vector<Obs_State> Obs_gather;

void graph_local(Plan_Trajectory Trajectory){
    local_path.header.frame_id="path";
    local_path.header.stamp=ros::Time::now();
    local_path.id=8888888;
    local_path.type=visualization_msgs::Marker::LINE_STRIP;
    local_path.action=visualization_msgs::Marker::ADD;
    local_path.scale.x=0.5;
    local_path.color.r=0.0;
    local_path.color.g=1.0;
    local_path.color.b=0.0;
    local_path.color.a=1.0;
    geometry_msgs::Point p;
    for (int i=0;i<Trajectory.trajectory_x.size();i++){
        if (!std::isnan(Trajectory.trajectory_x[i])){
            p.x=Trajectory.trajectory_x[i];
            p.y=Trajectory.trajectory_y[i];
            p.z=0.0;
            local_path.points.push_back(p);
        }
        
    }
    //ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    pub.publish(local_path);
    planed=true;
}

void graph_host(const Host_State &ego){
    Host_marker.header.frame_id="path";
    Host_marker.header.stamp=ros::Time::now();
    Host_marker.id=100;
    Host_marker.type=visualization_msgs::Marker::CUBE;
    Host_marker.action=visualization_msgs::Marker::ADD;
    Host_marker.scale.x=3.0;
    Host_marker.scale.y=1.63;
    Host_marker.scale.z=1.0;
    Host_marker.color.r=1.0;
    Host_marker.color.g=5.0;
    Host_marker.color.b=0.0;
    Host_marker.color.a=1.0;
    Host_marker.pose.position.x=ego.x;
    Host_marker.pose.position.y=ego.y;
    Host_marker.pose.position.z=0.0;
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, ego.heading_xy);
    Host_marker.pose.orientation.x = qtn.getX();
    Host_marker.pose.orientation.y = qtn.getY();
    Host_marker.pose.orientation.z = qtn.getZ();
    Host_marker.pose.orientation.w = qtn.getW();
}

void graph_obs(const vector<Obs_State> &Obss){
    Obs_marker.header.frame_id="path";
    Obs_marker.header.stamp=ros::Time::now();
    Obs_marker.id=99;
    Obs_marker.type=visualization_msgs::Marker::CUBE;
    Obs_marker.action=visualization_msgs::Marker::ADD;
    Obs_marker.scale.x=3.0;
    Obs_marker.scale.y=1.0;
    Obs_marker.scale.z=1.0;
    Obs_marker.color.r=1.0;
    Obs_marker.color.g=0.0;
    Obs_marker.color.b=0.0;
    Obs_marker.color.a=1.0;
    Obs_marker.pose.position.x=Obss[0].x;
    Obs_marker.pose.position.y=Obss[0].y;
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, Obss[0].heading);
    Obs_marker.pose.orientation.x = qtn.getX();
    Obs_marker.pose.orientation.y = qtn.getY();
    Obs_marker.pose.orientation.z = qtn.getZ();
    Obs_marker.pose.orientation.w = qtn.getW();

}

void graph_global_path(const vector<double>& global_x, const vector<double>& global_y){
    line_marker.header.frame_id="path";
    line_marker.header.stamp=ros::Time::now();
    //line_marker.ns="line";
    line_marker.id=1000;
    line_marker.type=visualization_msgs::Marker::LINE_STRIP;
    line_marker.action=visualization_msgs::Marker::ADD;
    line_marker.scale.x=0.1;
    line_marker.color.r=1.0;
    line_marker.color.g=0.0;
    line_marker.color.b=0.0;
    line_marker.color.a=1.0;
    geometry_msgs::Point p;
    for (int i=0;i<global_x.size();i++){
        p.x=global_x[i];
        p.y=global_y[i];
        p.z=0.0;
        
        line_marker.points.push_back(p);
    }
    
}

void Controller(const Plan_Trajectory& trajectory, Host_State& ego){
    int index=0;
    double dt=1.0/50.0;
    double min_dis=std::numeric_limits<double>::max();
    for (int i=0;i<trajectory.trajectory_x.size();i++){
        double dis=pow(ego.x-trajectory.trajectory_x[i],2) + 
                   pow(ego.y-trajectory.trajectory_y[i],2);
        if (dis < min_dis){
            min_dis=dis;
            index=i;
        }  
    }
    double xr=trajectory.trajectory_x[index];
    double yr=trajectory.trajectory_y[index];
    double thetar=trajectory.trajectory_heading[index];
    double kappar=trajectory.trajectory_kappa[index];
    double vr=trajectory.trajectory_velocity[index];
    double ar=trajectory.trajectory_accel[index];
    std::cout<<"xr=="<<xr;
    std::cout<<"  yr=="<<yr;
    std::cout<<"  vr=="<<vr;
    std::cout<<"  ar=="<<ar;
    std::cout<<"  headingr=="<<thetar;
    std::cout<<"  kappar="<<kappar<<std::endl;
    std::cout<<"host_x="<<ego.x<<"  ";
    std::cout<<"host_y="<<ego.y<<"  ";
    std::cout<<"host_heading="<<ego.heading_xy<<std::endl;
    double pre_x_ctrl=ego.x + ego.vx*dt*cos(ego.heading_xy) - ego.vy*dt*sin(ego.heading_xy);
    double pre_y_ctrl=ego.y + ego.vy*dt*cos(ego.heading_xy) + ego.vx*dt*sin(ego.heading_xy);
    double pre_yaw_ctrl=ego.heading_xy;
    double pre_vx_ctrl=ego.vx;
    double pre_vy_ctrl=ego.vy;
    //LQR
    LQR_Contrller.Init_Parameters(xr, yr, thetar, vr, kappar);
    auto controller=LQR_Contrller.solveLQR(pre_x_ctrl, pre_y_ctrl, pre_yaw_ctrl);
    vr=std::get<0>(controller);
    double wheel_steer=std::get<1>(controller);
    //PID
    double d_err[2]={pre_x_ctrl-xr , pre_y_ctrl-yr};
    double tor[2]={cos(thetar) , sin(thetar)};
    double nor[2]={-sin(thetar) , cos(thetar)};
    double es=d_err[0]*tor[0] + d_err[1]*tor[1];
    double ed=d_err[0]*nor[0] + d_err[1]*nor[1];
    double projection_point_thetar=thetar + kappar*es;
    double s_dot=(pre_vx_ctrl*cos(pre_yaw_ctrl-projection_point_thetar) - 
                pre_vy_ctrl*sin(pre_yaw_ctrl-projection_point_thetar))/(1-kappar*ed);
    double pid_out=PID_distance.Control(es, dt);
    double err=pid_out + vr - s_dot;
    if (err > 45) err=45;
    if (err < -45) err=-45;
    double accel=PID_speed.Control(err , dt);
    //update
    ego.ax=accel;
    if (wheel_steer > M_PI/4) wheel_steer=M_PI/4;
    if (wheel_steer < -M_PI/4) wheel_steer=-M_PI/4;
    if (accel > 6.0) accel=6.0;
    if (accel < -4.0) accel=-4.0;
    std::cout<<"steer=="<<wheel_steer<<"  accel=="<<accel<<std::endl;
    Ego_State_Update(ego, dt, wheel_steer);
}




vector<double> global_x;
vector<double> global_y;
vector<double> global_heading;
vector<double> global_kappa;

Host_State host;
FindHostOriginPoint Host_Proj;
FindHostOriginPoint Host_Origin;
FindHostOriginPoint Plan_Start_Origin;
Plan_Trajectory pre_trajectory;
Plan_Trajectory Trajectory_final;
// 获取当前时间点
auto current_ = std::chrono::system_clock::now();
// 将当前时间点转换为秒精度
auto seconds = std::chrono::time_point_cast<std::chrono::milliseconds>(current_);
// 获取秒数，并转换为 double
double global_time = static_cast<double>(seconds.time_since_epoch().count());
Static_Obs Handle_Obs;
Lattice Lattice_Planner(1e6, 2000, 2, 3, 10, 11, 4, 15, 1);
PATH_QP_Solver Path_QP_Planner(2, 25000, 50, 20, 15, 15, 15, 15, 1.0, 1.0, 1.63);
void EM_Planner(const ros::TimerEvent& event){
    
    
    //std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    
    pub.publish(line_marker);
    pub.publish(Obs_marker);
    // 获取当前时间点
    auto current_ms = std::chrono::system_clock::now();
    auto seconds_planner = std::chrono::time_point_cast<std::chrono::milliseconds>(current_ms);
    double current_time = static_cast<double>(seconds_planner.time_since_epoch().count());
    current_time-=global_time;
    current_time=current_time/1000.0;
    std::cout<<"planner_time"<<current_time<<std::endl;
    Host_Proj.ImplementFind(host.x, host.y, global_x, global_y,
                            global_heading, global_kappa);
    auto reference_init=GLOBAL_H::UnSmoothPath(Host_Proj.pre_match_point_index_set,
                        global_x, global_y);
    
    vector<double> reference_x_init=std::get<0>(reference_init);
    vector<double> reference_y_init=std::get<1>(reference_init);
    
    
    auto referenceline=Smooth_Globalpath(reference_x_init, reference_y_init);
    
    vector<double> referenceline_x=std::get<0>(referenceline);
    vector<double> referenceline_y=std::get<1>(referenceline);
    
    auto referenceline_head_kappa=Cal_heading_kappa(referenceline_x,referenceline_y);
    vector<double> referenceline_heading=std::get<0>(referenceline_head_kappa);
    vector<double> referenceline_kappa=std::get<1>(referenceline_head_kappa);
    //std::cout<<"host.x=="<<host.x<<std::endl;
    Host_Origin.ImplementFind(host.x, host.y, referenceline_x, referenceline_y,
                    referenceline_heading, referenceline_kappa);
    //std::cout<<"jjjjj=="<<Host_Origin.proj_x<<std::endl;
    vector<double> index2s=REFERENCELINE_H::Get_Index2s(referenceline_x, referenceline_y,
                                Host_Origin.proj_x, Host_Origin.proj_y, 
                                Host_Origin.pre_match_point_index_set);
    
    auto Start_And_Trajectory=
            REFERENCELINE_H::Calc_Plan_Start_Info_And_Stitch_Trajectory(pre_trajectory,
            current_time, host);
    
    Plan_Start_gcs plan_start=std::get<0>(Start_And_Trajectory);
    Plan_Trajectory stitch_trajectory=std::get<1>(Start_And_Trajectory);
    Plan_Start_Origin.ImplementFind(plan_start.x, plan_start.y, referenceline_x,
                        referenceline_y, referenceline_heading,referenceline_kappa);
    Proj_Origin_Point proj_plan_start(Plan_Start_Origin.proj_x, 
                        Plan_Start_Origin.proj_y, 
                        Plan_Start_Origin.proj_heading,
                        Plan_Start_Origin.proj_kappa,
                        Plan_Start_Origin.pre_match_point_index_set);
    //std::cout<<"xxxxxxx==="<<plan_start.x<<std::endl;
    auto plan_start_sl=REFERENCELINE_H::Cartesian2Frenet(plan_start, referenceline_x, 
        referenceline_y, proj_plan_start, index2s);
    double plan_start_s=std::get<0>(plan_start_sl);
    //std::cout<<"xxxx=="<<plan_start_s<<endl;
    double plan_start_l=std::get<1>(plan_start_sl);
    double plan_start_dl=std::get<2>(plan_start_sl);
    double plan_start_ddl=std::get<3>(plan_start_sl);
    Handle_Obs.Init_Obs();
    Handle_Obs.Handle_Obs(host, Obs_gather, referenceline_x, referenceline_y, 
            referenceline_heading,referenceline_kappa, index2s);
    auto Static_Obs=Handle_Obs.Get_SL();
    vector<double> static_obs_s_set=std::get<0>(Static_Obs);
    vector<double> static_obs_l_set=std::get<1>(Static_Obs);
    auto dp_path_init=Lattice_Planner.Dynamic_Planning(static_obs_s_set, 
                        static_obs_l_set,plan_start_s, plan_start_l,
                        plan_start_dl, plan_start_ddl);
    vector<double> dp_path_s_init=std::get<0>(dp_path_init);
    vector<double> dp_path_l_init=std::get<1>(dp_path_init);
    
    Lattice_Planner.Increase_DP_Path_Node(dp_path_s_init, dp_path_l_init, 
                        plan_start_s, plan_start_l, plan_start_dl, plan_start_ddl);
    auto dp_path_fianl=Lattice_Planner.Get_SLDLDDL();
    vector<double> dp_path_s_final=std::get<0>(dp_path_fianl);
    vector<double> dp_path_l_final=std::get<1>(dp_path_fianl);
    vector<double> dp_path_dl_final=std::get<2>(dp_path_fianl);
    vector<double> dp_path_ddl_final=std::get<3>(dp_path_fianl);
    
    Path_QP_Planner.Generate_Convex_Space(dp_path_s_final, dp_path_l_final, 
                    static_obs_s_set, static_obs_l_set);
    Path_QP_Planner.Path_Planning_With_Quadratic_Programming(plan_start_s, plan_start_l,
                                plan_start_dl, plan_start_ddl);
    Path_QP_Planner.Increase_QP_Node();
    
    auto qp_path_final=Path_QP_Planner.Get_QP_Result();
    vector<double> qp_path_s_final=std::get<0>(qp_path_final);
    vector<double> qp_path_l_final=std::get<1>(qp_path_final);
    vector<double> qp_path_dl_final=std::get<2>(qp_path_final);
    vector<double> qp_path_ddl_final=std::get<3>(qp_path_final);
    
    auto trajectory_init=FRETOCAR_H::FrenetToCartesian(qp_path_s_final, 
            qp_path_l_final, qp_path_dl_final, 
            qp_path_ddl_final, referenceline_x, referenceline_y, referenceline_heading, 
            referenceline_kappa, index2s);
    
    vector<double> trajectory_x_init=std::get<0>(trajectory_init);
    
    vector<double> trajectory_y_init=std::get<1>(trajectory_init);
    vector<double> trajectory_heading_init=std::get<2>(trajectory_init);
    vector<double> trajectory_kappa_init=std::get<3>(trajectory_init);
    auto s_and_end=SPEED_H::Calc_Path_Length_And_Map_Between_Point_And_S(trajectory_x_init, trajectory_y_init);
    double path_s_end=std::get<0>(s_and_end);
    vector<double> path_s=std::get<1>(s_and_end);
    
    auto sdot_sdot2=SPEED_H::Calc_Speed_Planning_Start_Condition(plan_start.vx, 
                    plan_start.vy, plan_start.ax, plan_start.ay, plan_start.heading);
    double plan_start_s_dot=std::get<0>(sdot_sdot2);
    double plan_start_s_dot2=std::get<1>(sdot_sdot2);
    
    auto s_sdot_dot2_time_init=SPEED_H::Sample_Speed_Qp_Planning(plan_start_s_dot,
        plan_start_s_dot2, path_s_end, 6.0);
    
    vector<double> s_init=std::get<0>(s_sdot_dot2_time_init);
    // for (int i=0;i<s_init.size();i++){
    //     std::cout<<s_init[i]<<std::endl;
    // }
    vector<double> s_dot_init=std::get<1>(s_sdot_dot2_time_init);
    vector<double> s_dot2_init=std::get<2>(s_sdot_dot2_time_init);
    vector<double> relative_time_init=std::get<3>(s_sdot_dot2_time_init);
    auto s_sdot_dot2_time=SPEED_H::Increase_St_Point_Count(s_init, s_dot_init, 
                                                s_dot2_init, relative_time_init);
    vector<double> s=std::get<0>(s_sdot_dot2_time);
    vector<double> s_dot=std::get<1>(s_sdot_dot2_time);
    vector<double> s_dot2=std::get<2>(s_sdot_dot2_time);
    vector<double> relative_time=std::get<3>(s_sdot_dot2_time);
    Plan_Trajectory trajectory=SPEED_H::Path_And_Speed_Merge(s,s_dot,s_dot2, 
                relative_time,plan_start.start_time, path_s, trajectory_x_init, 
                trajectory_y_init, trajectory_heading_init, trajectory_kappa_init);
    
    Trajectory_final=SPEED_H::Stitch_Trajectory(trajectory, stitch_trajectory);
    pre_trajectory=Trajectory_final;
    graph_local(Trajectory_final);
    // for (int i=0;i<Trajectory_final.trajectory_x.size();i++){
    //     std::cout<<Trajectory_final.trajectory_x[i]<<"   ";
    //     std::cout<<Trajectory_final.trajectory_y[i]<<std::endl;
    // }
    
    // 等待200ms
    
    
    
}
void Control(const ros::TimerEvent& event){
    
    Controller(Trajectory_final, host);
    graph_host(host);
    pub.publish(Host_marker);
    
}
void VisualizaPath(const ros::TimerEvent& event){
    pub.publish(local_path);
}


int main(int argc, char *argv[]){

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"emplanner");
    ros::NodeHandle nh;
    pub=nh.advertise<visualization_msgs::Marker>("visualization_marker",100);
    

    Obs_State obs;
    obs.x=20.0;
    obs.y=0.0;
    obs.heading=0.0;
    obs.velocity=0.0;
    Obs_gather.push_back(obs);

    auto globalpath=generate_globalpath();
    global_x=std::get<0>(globalpath);
    global_y=std::get<1>(globalpath);
    auto heading_kappa=Cal_heading_kappa(global_x,global_y);
    global_heading=std::get<0>(heading_kappa);
    global_kappa=std::get<1>(heading_kappa);

    graph_global_path(global_x, global_y);
    graph_obs(Obs_gather);
    for (int i=0; i<3;i++){
        ros::Rate(5).sleep();
        pub.publish(line_marker);
        pub.publish(Obs_marker);
    }
    
    
    ros::Timer timer_planner = nh.createTimer(ros::Duration(0.2), EM_Planner);
    ros::Timer timer_controller = nh.createTimer(ros::Duration(0.02), Control);
    ros::Timer timer_visualizapath = nh.createTimer(ros::Duration(0.01), VisualizaPath);
    ros::spin();
    

    

    // // 启动两个线程
    // std::thread PLAN(EM_Planner);
    // std::thread CONTROL(Control);

    // // 等待用户输入或其他条件来通知线程退出
    // std::cout << "Press Enter to stop the thread." << std::endl;
    // std::cin.get();

    // // 设置标志以通知线程退出
    


    // // 结束线程
    // PLAN.join();
    // CONTROL.join();





    return 0;
}