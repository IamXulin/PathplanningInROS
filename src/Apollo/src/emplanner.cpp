#include "emplanner.h"

//visualization_msgs::Marker line_marker;
visualization_msgs::Marker Obs_marker;
visualization_msgs::Marker Host_marker;
visualization_msgs::Marker local_path;
visualization_msgs::Marker global_path;

auto global_time = std::chrono::system_clock::now();

Path_planning_node::Path_planning_node() {};
Path_planning_node::~Path_planning_node() {};

bool Path_planning_node::Init(){
    auto globalpath=generate_globalpath();
    global_x=std::get<0>(globalpath);
    global_y=std::get<1>(globalpath);
    auto heading_kappa=Cal_heading_kappa(global_x,global_y);
    global_heading=std::get<0>(heading_kappa);
    global_kappa=std::get<1>(heading_kappa);

    lqrController_=std::shared_ptr<LQRController>(new LQRController());
    speedPidControllerPtr_=std::shared_ptr<PIDController> (new PIDController(0.4, 0.1, 0.0));
    locationPidControllerPtr_=std::shared_ptr<PIDController> (new PIDController(0.4, 0.1, 0.0));;

    graph_global_path();
    Host_Proj=std::shared_ptr<FindHostOriginPoint>(new FindHostOriginPoint());
    Host_Origin=std::shared_ptr<FindHostOriginPoint>(new FindHostOriginPoint());
    Plan_Start_Origin=std::shared_ptr<FindHostOriginPoint>(new FindHostOriginPoint());
    Handle_Obs=std::shared_ptr<Static_Obs>(new Static_Obs());
    Lattice_Planner=std::shared_ptr<Lattice>(new Lattice(1e6, 2000, 2, 3, 10, 11, 4, 15, 1));
    Path_QP_Planner=std::shared_ptr<PATH_QP_Solver>(
        new PATH_QP_Solver(2, 25000, 50, 20, 15, 15, 15, 15, 1.0, 1.0, 1.63));
    UpdateObs();
    // 获取当前时间点
    
    
    ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    visTimer_=nh_.createTimer(ros::Duration(1/vis_frequency_),
                    &Path_planning_node::VisualizaPath,this);
    plannerTimer_=nh_.createTimer(ros::Duration(1/plannerFrequency_),
                    &Path_planning_node::EM_Planner,this);
    controlTimer_=nh_.createTimer(ros::Duration(1/controlFrequency_),
                    &Path_planning_node::Control,this);
    return true;
}

void Path_planning_node::VisualizaPath(const ros::TimerEvent& event){
    graph_obs();
    graph_local();
    graph_host();
}

void Path_planning_node::UpdateObs(){
    Obs_State obs1;
    obs1.x=30;
    obs1.y=0.0;
    obs1.heading=0.0;
    obs1.velocity=0.0;
    Obs_gather.push_back(obs1);
}

void Path_planning_node::EM_Planner(const ros::TimerEvent& event){
    auto now = std::chrono::system_clock::now();
    // 将当前时间点转换为毫秒数
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - global_time);
    auto ms_count = duration.count();
    // 将毫秒数转换为 double 类型
    double ms_double = static_cast<double>(ms_count);
    double current_time=ms_double/1000.0;
    std::cout<<"planner_time"<<current_time<<std::endl;
    Host_Proj->ImplementFind(host.x, host.y, global_x, global_y,
                            global_heading, global_kappa);
    auto reference_init=UnSmoothPath(Host_Proj->pre_match_point_index_set,
                        global_x, global_y);
    vector<double> reference_x_init=std::get<0>(reference_init);
    vector<double> reference_y_init=std::get<1>(reference_init);

    auto referenceline=Smooth_Globalpath(reference_x_init, reference_y_init);
    vector<double> referenceline_x=std::get<0>(referenceline);
    vector<double> referenceline_y=std::get<1>(referenceline);

    auto referenceline_head_kappa=Cal_heading_kappa(referenceline_x,referenceline_y);
    vector<double> referenceline_heading=std::get<0>(referenceline_head_kappa);
    vector<double> referenceline_kappa=std::get<1>(referenceline_head_kappa);

    Host_Origin->ImplementFind(host.x, host.y, referenceline_x, referenceline_y,
                    referenceline_heading, referenceline_kappa);

    vector<double> index2s=Get_Index2s(referenceline_x, referenceline_y,
                                Host_Origin->proj_x, Host_Origin->proj_y, 
                                Host_Origin->pre_match_point_index_set);

    auto Start_And_Trajectory=Calc_Plan_Start_Info_And_Stitch_Trajectory(pre_trajectory,
            current_time, host);
    Plan_Start_gcs plan_start=std::get<0>(Start_And_Trajectory);
    Plan_Trajectory stitch_trajectory=std::get<1>(Start_And_Trajectory);

    Plan_Start_Origin->ImplementFind(plan_start.x, plan_start.y, referenceline_x,
                        referenceline_y, referenceline_heading,referenceline_kappa);
    Proj_Origin_Point proj_plan_start(Plan_Start_Origin->proj_x, 
                        Plan_Start_Origin->proj_y, 
                        Plan_Start_Origin->proj_heading,
                        Plan_Start_Origin->proj_kappa,
                        Plan_Start_Origin->pre_match_point_index_set);
    auto plan_start_sl=Cartesian2Frenet(plan_start, referenceline_x, 
        referenceline_y, proj_plan_start, index2s);
    double plan_start_s=std::get<0>(plan_start_sl);
    //std::cout<<"xxxx=="<<plan_start_s<<endl;
    double plan_start_l=std::get<1>(plan_start_sl);
    double plan_start_dl=std::get<2>(plan_start_sl);
    double plan_start_ddl=std::get<3>(plan_start_sl);

    Handle_Obs->Init_Obs();
    Handle_Obs->Handle_Obs(host, Obs_gather, referenceline_x, referenceline_y, 
            referenceline_heading,referenceline_kappa, index2s);

    auto Static_Obs=Handle_Obs->Get_SL();
    vector<double> static_obs_s_set=std::get<0>(Static_Obs);
    vector<double> static_obs_l_set=std::get<1>(Static_Obs);
    auto dp_path_init=Lattice_Planner->Dynamic_Planning(static_obs_s_set, 
                        static_obs_l_set,plan_start_s, plan_start_l,
                        plan_start_dl, plan_start_ddl);
    vector<double> dp_path_s_init=std::get<0>(dp_path_init);
    vector<double> dp_path_l_init=std::get<1>(dp_path_init);
    Lattice_Planner->Increase_DP_Path_Node(dp_path_s_init, dp_path_l_init, 
                        plan_start_s, plan_start_l, plan_start_dl, plan_start_ddl);
    auto dp_path_fianl=Lattice_Planner->Get_SLDLDDL();
    vector<double> dp_path_s_final=std::get<0>(dp_path_fianl);
    vector<double> dp_path_l_final=std::get<1>(dp_path_fianl);
    vector<double> dp_path_dl_final=std::get<2>(dp_path_fianl);
    vector<double> dp_path_ddl_final=std::get<3>(dp_path_fianl);

    Path_QP_Planner->Generate_Convex_Space(dp_path_s_final, dp_path_l_final, 
                    static_obs_s_set, static_obs_l_set);
    Path_QP_Planner->Path_Planning_With_Quadratic_Programming(plan_start_s, plan_start_l,
                                plan_start_dl, plan_start_ddl);
    Path_QP_Planner->Increase_QP_Node();
    auto qp_path_final=Path_QP_Planner->Get_QP_Result();
    vector<double> qp_path_s_final=std::get<0>(qp_path_final);
    vector<double> qp_path_l_final=std::get<1>(qp_path_final);
    vector<double> qp_path_dl_final=std::get<2>(qp_path_final);
    vector<double> qp_path_ddl_final=std::get<3>(qp_path_final);

    auto trajectory_init=FrenetToCartesian(qp_path_s_final, 
            qp_path_l_final, qp_path_dl_final, 
            qp_path_ddl_final, referenceline_x, referenceline_y, referenceline_heading, 
            referenceline_kappa, index2s);
    vector<double> trajectory_x_init=std::get<0>(trajectory_init);
    vector<double> trajectory_y_init=std::get<1>(trajectory_init);
    vector<double> trajectory_heading_init=std::get<2>(trajectory_init);
    vector<double> trajectory_kappa_init=std::get<3>(trajectory_init);

    auto s_and_end=Calc_Path_Length_And_Map_Between_Point_And_S(trajectory_x_init, trajectory_y_init);
    double path_s_end=std::get<0>(s_and_end);
    vector<double> path_s=std::get<1>(s_and_end);

    auto sdot_sdot2=Calc_Speed_Planning_Start_Condition(plan_start.vx, 
                    plan_start.vy, plan_start.ax, plan_start.ay, plan_start.heading);
    double plan_start_s_dot=std::get<0>(sdot_sdot2);
    double plan_start_s_dot2=std::get<1>(sdot_sdot2);
    
    auto s_sdot_dot2_time_init=Sample_Speed_Qp_Planning(plan_start_s_dot,
        plan_start_s_dot2, path_s_end, 6.0);
    vector<double> s_init=std::get<0>(s_sdot_dot2_time_init);
    vector<double> s_dot_init=std::get<1>(s_sdot_dot2_time_init);
    vector<double> s_dot2_init=std::get<2>(s_sdot_dot2_time_init);
    vector<double> relative_time_init=std::get<3>(s_sdot_dot2_time_init);
    auto s_sdot_dot2_time=Increase_St_Point_Count(s_init, s_dot_init, 
                                                s_dot2_init, relative_time_init);
    vector<double> s=std::get<0>(s_sdot_dot2_time);
    vector<double> s_dot=std::get<1>(s_sdot_dot2_time);
    vector<double> s_dot2=std::get<2>(s_sdot_dot2_time);
    vector<double> relative_time=std::get<3>(s_sdot_dot2_time);
    Plan_Trajectory trajectory=Path_And_Speed_Merge(s,s_dot,s_dot2, 
                relative_time,plan_start.start_time, path_s, trajectory_x_init, 
                trajectory_y_init, trajectory_heading_init, trajectory_kappa_init);
    
    Trajectory_final=Stitch_Trajectory(trajectory, stitch_trajectory);
    pre_trajectory=Trajectory_final;
    pland=true;
    init_run=false;

}

void Path_planning_node::Control(const ros::TimerEvent& event){
    double dt=0.01;
    double xr, yr, thetar, kappar, vr, ar;
    auto now = std::chrono::system_clock::now();
    // 将当前时间点转换为毫秒数
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - global_time);
    auto ms_count = duration.count();
    // 将毫秒数转换为 double 类型
    double ms_double = static_cast<double>(ms_count);
    double current_time=ms_double/1000.0;
    if (init_run){
        xr=0;
        yr=0;
        thetar=0;
        kappar=0;
        vr=0;
        ar=0;
    }else{
        if (pland){
            double control_time = current_time + 0.01;
            start_index=0;
            while (Trajectory_final.trajectory_time[start_index]==-1) start_index++;
            if (control_time>Trajectory_final.trajectory_time[start_index] && 
                            Trajectory_final.trajectory_time[start_index]!=0){
                int ii=start_index;
                for (int kk=start_index;kk<Trajectory_final.trajectory_time.size();kk++){
                    if (Trajectory_final.trajectory_time[kk]>control_time){
                        ii=kk;
                        break;
                    }
                }
                xr=Trajectory_final.trajectory_x[ii];
                yr=Trajectory_final.trajectory_y[ii];
                thetar=Trajectory_final.trajectory_heading[ii];
                kappar=Trajectory_final.trajectory_kappa[ii];
                vr=Trajectory_final.trajectory_velocity[ii];
                ar=Trajectory_final.trajectory_accel[ii];

            }
        }else{
            double control_time = current_time + 0.01;
            if (control_time>pre_trajectory.trajectory_time[start_index] && 
                            pre_trajectory.trajectory_time[start_index]!=0){
                int ii=start_index;
                for (int kk=start_index;kk<pre_trajectory.trajectory_time.size();kk++){
                    if (pre_trajectory.trajectory_time[kk]>control_time){
                        ii=kk;
                        break;
                    }
                }
                xr=pre_trajectory.trajectory_x[ii];
                yr=pre_trajectory.trajectory_y[ii];
                thetar=pre_trajectory.trajectory_heading[ii];
                kappar=pre_trajectory.trajectory_kappa[ii];
                vr=pre_trajectory.trajectory_velocity[ii];
                ar=pre_trajectory.trajectory_accel[ii];

            }
        }
    }
 
    std::cout<<"xr=="<<xr;
    std::cout<<"  yr=="<<yr;
    std::cout<<"  vr=="<<vr;
    std::cout<<"  ar=="<<ar;
    std::cout<<"  headingr=="<<thetar;
    std::cout<<"  kappar="<<kappar<<std::endl;
    std::cout<<"host_x="<<host.x<<"  ";
    std::cout<<"host_y="<<host.y<<"  ";
    std::cout<<"host_heading="<<host.heading_xy<<std::endl;
    double pre_x_ctrl=host.x + host.vx*dt*cos(host.heading_xy) - host.vy*dt*sin(host.heading_xy);
    double pre_y_ctrl=host.y + host.vy*dt*cos(host.heading_xy) + host.vx*dt*sin(host.heading_xy);
    double pre_yaw_ctrl=host.heading_xy;
    double pre_vx_ctrl=host.vx;
    double pre_vy_ctrl=host.vy;
    //LQR
    
    lqrController_->Init_Parameters(xr, yr, thetar, vr, kappar);
    auto controller=lqrController_->solveLQR(pre_x_ctrl, pre_y_ctrl, pre_yaw_ctrl);
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
    double pid_out=locationPidControllerPtr_->Control(es, 0.01);
    double err=pid_out + vr - s_dot;
    if (err > 45) err=45;
    if (err < -45) err=-45;
    double accel=speedPidControllerPtr_->Control(err , dt);
    //update
    host.ax=accel;
    if (wheel_steer > M_PI/4) wheel_steer=M_PI/4;
    if (wheel_steer < -M_PI/4) wheel_steer=-M_PI/4;
    if (accel > 6.0) accel=6.0;
    if (accel < -4.0) accel=-4.0;
    std::cout<<"steer=="<<wheel_steer<<"  accel=="<<accel<<std::endl;
    Ego_State_Update(host, 0.01, wheel_steer);
}

void Path_planning_node::graph_obs(){
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
    Obs_marker.pose.position.x=Obs_gather[0].x;
    Obs_marker.pose.position.y=Obs_gather[0].y;
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, Obs_gather[0].heading);
    Obs_marker.pose.orientation.x = qtn.getX();
    Obs_marker.pose.orientation.y = qtn.getY();
    Obs_marker.pose.orientation.z = qtn.getZ();
    Obs_marker.pose.orientation.w = qtn.getW();
    pub_marker.publish(Obs_marker);
}

void Path_planning_node::graph_global_path(){
    global_path.header.frame_id="path";
    global_path.header.stamp=ros::Time::now();
    global_path.id=888888888;
    global_path.type=visualization_msgs::Marker::LINE_STRIP;
    global_path.action=visualization_msgs::Marker::ADD;
    global_path.scale.x=0.5;
    global_path.color.r=0.0;
    global_path.color.g=1.0;
    global_path.color.b=0.0;
    global_path.color.a=1.0;
    geometry_msgs::Point p;
    for (int i=0;i<global_x.size();i++){
        p.x=global_x[i];
        p.y=global_y[i];
        p.z=0.0;
        global_path.points.push_back(p);
    }
    pub_marker.publish(global_path);
}

void Path_planning_node::graph_local(){
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
    for (int i=0;i<Trajectory_final.trajectory_x.size();i++){
        if (!std::isnan(Trajectory_final.trajectory_x[i])){
            p.x=Trajectory_final.trajectory_x[i];
            p.y=Trajectory_final.trajectory_y[i];
            p.z=0.0;
            local_path.points.push_back(p);
        }
        
    }
    pub_marker.publish(local_path);
}

void Path_planning_node::graph_host(){
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
    Host_marker.pose.position.x=host.x;
    Host_marker.pose.position.y=host.y;
    Host_marker.pose.position.z=0.0;
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, host.heading_xy);
    Host_marker.pose.orientation.x = qtn.getX();
    Host_marker.pose.orientation.y = qtn.getY();
    Host_marker.pose.orientation.z = qtn.getZ();
    Host_marker.pose.orientation.w = qtn.getW();
    pub_marker.publish(Host_marker);
}