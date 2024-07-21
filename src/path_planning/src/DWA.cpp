#include"DWA.h"



void DWA::plot_path(vector<State> trajectory){


    pub_SG.publish(start);
    pub_SG.publish(goal);
    Path.action = visualization_msgs::Marker::ADD;
    for (int i=0;i<trajectory.size();i++){
        geometry_msgs::Point p;
        p.x = trajectory[i].x;
        p.y = trajectory[i].y;
        p.z = 0.3;
        Path.points.push_back(p);
    }
    publi.publish(Path);
 

}




vector<State> trajectory_predict(State state_init,double v, double w,
                            double predict_time, double dt){
    vector<State> trajectory;
    State state=state_init;
    trajectory.push_back(state);
    double time=0;
    while (time<predict_time){
        State sta=KinematicModel(state, v, w,dt);
        trajectory.push_back(sta);
        state=sta;
        //ROS_INFO("state=[%.2f,%.2f,%.2f,%.2f,%.2f]",sta.x,sta.y,sta.yaw,sta.V,sta.W);
        time+=dt;
    }
    return trajectory;
}

State KinematicModel(State state,double v, double w,double dt){
    State sta;
    sta.x=state.x+v*cos(state.yaw)*dt;
    sta.y=state.y+v*sin(state.yaw)*dt;
    sta.yaw=state.yaw+w*dt;
    sta.V=v;
    sta.W=w;
    return sta;
}

void DWA::initial_obs(){
    std::random_device seed;
    std::mt19937 prob(seed());
    std::uniform_int_distribution<> dospow(0,map_w-1);
    std::uniform_int_distribution<> dospoh(0,map_h-1);
    int id=0;
    for (int i=0;i<map_w;i++){
        for (int j=0;j<map_h;j++){
            double ranNum=static_cast<double> (dospow(prob))/(map_w-1);
            Obs obs;
            if (ranNum<0.05 || i==0 || i==map_w-1 || j==0 || j==map_h-1){
                obs.x=i;
                obs.y=j;
                full_MarkerArray(i,j,id,0.5,0.5,0.0,1.0,0.0,0.0,1.0,"obs");
                Obstacle.push_back(obs);
                id++;
            }
            

        }
    }
}

void DWA::full_MarkerArray(int x,int y,int id,double scale_x,double scale_y,
                        double scale_z,double color_r,double color_g,
                        double color_b,double color_a,string choice){
    visualization_msgs::Marker obs;
    obs.header.stamp = ros::Time::now();
    obs.header.frame_id="path";
    // 设置 ID
    obs.id = id;
    // 设置类型为立方体
    obs.type = visualization_msgs::Marker::CUBE;
    // 设置尺寸
    obs.scale.x = scale_x;
    obs.scale.y = scale_y;
    obs.scale.z = scale_z;
    // 设置颜色，这里为红色
    obs.color.r = color_r;
    obs.color.g = color_g;
    obs.color.b = color_b;
    obs.color.a = color_a; // 不透明
    // 设置位置
    obs.pose.position.x = x;
    obs.pose.position.y = y;
    obs.pose.position.z = 0.0;
    obs.pose.orientation.x=0.0;
    obs.pose.orientation.y=0.0;
    obs.pose.orientation.z=0.0;
    obs.pose.orientation.w=0.0;
    if (choice=="obs"){
        Obs_gather.markers.push_back(obs);
    }else if (choice=="start"){
        start=obs;
    }else if (choice=="goal"){
        goal=obs;
    }
}

void DWA::Advertise_start(int init_x,int init_y){
    start_x=init_x;
    start_y=init_y;
    ego_state.x=init_x;
    ego_state.y=init_y;
    ego_state.yaw=0.0;
    ego_state.V=0.0;
    ego_state.W=0.0;

    ego_control.v=0.0;
    ego_control.w=0.0;
    full_MarkerArray(start_x,start_y,-2,1.0,1.0,0.3,1.0,5.0,0.0,1.0,"start");
    pub_SG.publish(start);


    ego.header.stamp = ros::Time::now();
    ego.header.frame_id="path";
    ego.ns = "circle";
    ego.id = 0;
    ego.type = visualization_msgs::Marker::CYLINDER;
    ego.action = visualization_msgs::Marker::ADD;
    ego.pose.orientation.w = 1.0;
    ego.scale.x = 1.0;  // Diameter of the circle
    ego.scale.y = 1.0;  // Diameter of the circle
    ego.scale.z = 0.5; // Height of the cylinder (thickness of the circle)
    
    // Set position of the circle
    ego.pose.position.x = init_x;
    ego.pose.position.y = init_y;
    ego.pose.position.z = 0.0;

    // Set color of the circle
    ego.color.r = 1.0;
    ego.color.g = 5.0;
    ego.color.b = 0.5;
    ego.color.a = 1.0;
    pub_SG.publish(ego);

    Path.header.frame_id = "path";
    Path.header.stamp = ros::Time::now();
    Path.ns = "line";
    Path.id = 0;
    Path.type = visualization_msgs::Marker::LINE_STRIP;
    Path.action = visualization_msgs::Marker::ADD;
    Path.pose.orientation.w = 1.0;
    Path.scale.x = 0.05;  // Line width
    Path.scale.z=0.5;
    Path.color.r = 0.0;
    Path.color.g = 1.0;
    Path.color.b = 0.0;
    Path.color.a = 1.0;
}
void DWA::Advertise_goal(int end_x,int end_y){
    goal_x=end_x;
    goal_y=end_y;
    full_MarkerArray(goal_x,goal_y,-1,1.0,1.0,0.3,1.0,5.0,0.0,1.0,"goal");
    pub_SG.publish(goal);
}


DWA::DWA(ros::NodeHandle &nh){
    
    publi=nh.advertise<visualization_msgs::Marker>("/visualization_marker",10);
    pub=nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",10);
    pub_SG=nh.advertise<visualization_msgs::Marker>("/visualization_marker",10);

    
    ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    
    initial_obs();
    
    pub_obs();



    
}


void DWA::pub_obs(){
    ros::Rate t(5);
    for (int i=0;i<3;i++){
        pub.publish(Obs_gather);
        t.sleep();
    }
    
    
    
}


void DWA::state_update(){
    ego_state.x+=ego_control.v*cos(ego_state.yaw)*dt;
    ego_state.y+=ego_control.v*sin(ego_state.yaw)*dt;
    ego_state.yaw+=ego_control.w*dt;
    ego_state.V=ego_control.v;
    ego_state.W=ego_control.w;

    ego.DELETE;

    ego.header.stamp = ros::Time::now();
    ego.header.frame_id="path";
    ego.ns = "circle";
    ego.id = 0;
    ego.type = visualization_msgs::Marker::CYLINDER;
    ego.action = visualization_msgs::Marker::ADD;
    ego.pose.orientation.w = 1.0;
    ego.scale.x = 1.0;  // Diameter of the circle
    ego.scale.y = 1.0;  // Diameter of the circle
    ego.scale.z = 0.5; // Height of the cylinder (thickness of the circle)
    
    // Set position of the circle
    ego.pose.position.x = ego_state.x;
    ego.pose.position.y = ego_state.y;
    ego.pose.position.z = 0.0;

    // Set color of the circle
    ego.color.r = 1.0;
    ego.color.g = 5.0;
    ego.color.b = 0.5;
    ego.color.a = 1.0;
    pub_SG.publish(ego);

}

double DWA::eva_heading(vector<State> trajectory){
    double dx=goal_x-trajectory[trajectory.size()-1].x;
    double dy=goal_y-trajectory[trajectory.size()-1].y;
    double error_angle=atan2(dy,dx);
    //if (dy==0 && dx>0) error_angle=0;
    //if (dy==0 && dx<0) error_angle=-M_PI;
    if (trajectory[trajectory.size()-1].yaw > M_PI){
        trajectory[trajectory.size()-1].yaw-=2*M_PI;
    } 
    if (trajectory[trajectory.size()-1].yaw < -M_PI){
        trajectory[trajectory.size()-1].yaw+=2*M_PI;
    }
    // vector<double> selfyaw={cos(trajectory[trajectory.size()-1].yaw),
    //                         sin(trajectory[trajectory.size()-1].yaw)};
    // vector<double> self2goal={dx,dy};
    // double cost_angle=acos((selfyaw[0]*self2goal[0])/(sqrt(pow(dx,2)+pow(dy,2))));
    double cost_angle=error_angle-trajectory[trajectory.size()-1].yaw;
     

    double cost=M_PI-abs(cost_angle);
    return cost;
}

double DWA::eva_dist(vector<State> trajectory){
    double dis=judge_distance;
    for (int i=0;i<trajectory.size();i++){
        for (int j=0;j<Obstacle.size();j++){
            double D=sqrt(pow(trajectory[i].x-Obstacle[j].x,2)+
                    pow(trajectory[i].y-Obstacle[j].y,2));
            if (D<dis) dis=D;
        }
    }
    ROS_INFO("dis==%.2f",dis);
    return dis;
}

double DWA::eva_velocity(vector<State> trajectory){
    return trajectory[trajectory.size()-1].V;
}

vector<State> DWA::trajectory_evaluation(){
    
    double G_max=-1000000;
    vector<State> trajectory_opt;
    Control control_opt=ego_control;
    double good_heading;
    double good_dist;
    double good_evl;
    vector<double> dynamic_window=cal_dynamic_window_vel(ego_state.V,ego_state.W);
    for (double v=dynamic_window[0];v<=dynamic_window[1];v+=v_sample){
        
        for (double w=dynamic_window[2];w<=dynamic_window[3];w+=w_sample){
            vector<State> trajectory=trajectory_predict(ego_state,v, 
                            w,predict_time, dt);
            
            // ROS_INFO("trajectory[-1].x==%.2f",trajectory[-1].x);
            // ROS_INFO("trajectory[-1].y==%.2f",trajectory[-1].y);
            double dist=sqrt(pow(trajectory[trajectory.size()-1].x -goal_x,2)+
                            pow(trajectory[trajectory.size()-1].y-goal_y,2));
            //ROS_INFO("len_tra==%.d",trajectory.size());
            double gamma_;
            ROS_INFO("dist==%.2f",dist);
            if (dist > 2.5){
                gamma_=4;
            }else{
                gamma_=1.5;
            }
            ROS_INFO("goal_x==%.d  goal_y==%.d",goal_x,goal_y);
            ROS_INFO("xx=%.2f  yy=%.2f",trajectory[trajectory.size()-1].x,
                                    trajectory[trajectory.size()-1].y);
            ROS_INFO("gamma_==%.2f",gamma_);
            ROS_INFO("yaw==%.2f",ego_state.yaw);
            double heading_eval=alpha*eva_heading(trajectory);
            double dist_eval=beta*eva_dist(trajectory);
            double evl_eval=gamma*gamma_*eva_velocity(trajectory);
            double G=heading_eval+dist_eval+evl_eval;
            //ROS_INFO("heading_eval==%.2f dis_eval==%.2f evl_eval==%.2f",
            //heading_eval,dist_eval,evl_eval);
            //ROS_INFO("V的范围是%.2f-----%.2f",dynamic_window[0],dynamic_window[1]);
            //ROS_INFO("W的范围是%.2f-----%.2f",dynamic_window[2],dynamic_window[3]);
            //ROS_INFO("G==%.2f",G);
            //ROS_INFO("curent_v====%.2f    current_w=====%.2f",v,w);
            if (G>G_max){
                G_max=G;
                trajectory_opt=trajectory;
                control_opt.v=v;
                control_opt.w=w;
                good_heading=heading_eval;
                good_dist=dist_eval;
                good_evl=evl_eval;
            }
        }
    }
    //ROS_INFO("v====%.2f    w=====%.2f",control_opt.v,control_opt.w);
    //ROS_INFO("good_heading==%.2f  good_dist==%.2f  good_evl==%.2f",
                //good_heading,good_dist,good_evl);
    ego_control=control_opt;
    ROS_INFO("W==%.2f",ego_control.w);
    plot_path(trajectory_opt);
    //publi.publish(Path);
    return trajectory_opt;


}


vector<double> DWA::cal_dynamic_window_vel(double v,double w){
    vector<double> window={v_min,v_max,w_min,w_max};
    vector<double> Vm=cal_vel_limit();
    //ROS_INFO("Vm==[%.2f, %.2f, %.2f, %.2f]",Vm[0],Vm[1],Vm[2],Vm[3]);
    vector<double> Vd=cal_accel_limit(v,w);
    //ROS_INFO("Vd==[%.2f, %.2f, %.2f, %.2f]",Vd[0],Vd[1],Vd[2],Vd[3]);
    vector<double> Va=cal_obstacle_limit();
    //ROS_INFO("Va==[%.2f, %.2f, %.2f, %.2f]",Va[0],Va[1],Va[2],Va[3]);
    vector<double> A={Vm[0],Vd[0],Va[0]};
    vector<double> B={Vm[1],Vd[1],Va[1]};
    vector<double> C={Vm[2],Vd[2],Va[2]};
    vector<double> D={Vm[3],Vd[3],Va[3]};
    auto min_v=max_element(A.begin(),A.end());
    auto max_v=min_element(B.begin(),B.end());
    auto min_w=max_element(C.begin(),C.end());
    auto max_w=min_element(D.begin(),D.end());
    if (min_v != A.end()) window[0]=*min_v;
    if (max_v != B.end()) window[1]=*max_v;
    if (min_w != C.end()) window[2]=*min_w;
    if (max_w != D.end()) window[3]=*max_w;
    return window;
}

vector<double> DWA::cal_vel_limit(){
    vector<double> vel_limit={v_min,v_max,w_min,w_max};
    return vel_limit;
}
vector<double> DWA::cal_accel_limit(double v, double w){
    
    double v_low=v-a_vmax*dt;
    double v_high=v+a_vmax*dt;
    double w_low=w-a_wmax*dt;
    double w_high=w+a_wmax*dt;
    vector<double> accel_limit={v_low,v_high,w_low,w_high};
    return accel_limit;
}
vector<double> DWA::cal_obstacle_limit(){
    double v_low=v_min;
    double dis=obs_mindis();
    //ROS_INFO("99999999999999");
    double v_high=sqrt(2*dis*a_vmax);
    double w_low=w_min;
    double w_high=sqrt(2*dis*a_wmax);
    vector<double> obstacle_limit={v_low,v_high,w_low,w_high};
    return obstacle_limit;
}
double DWA::obs_mindis(){
    double dis=sqrt(pow(ego_state.x-Obstacle[0].x,2)+
                pow(ego_state.y-Obstacle[0].y,2));
    for (int j=1;j<Obstacle.size();j++){
        double D=sqrt(pow(ego_state.x-Obstacle[j].x,2)+
                pow(ego_state.y-Obstacle[j].y,2));
        if (D<dis) dis=D;
    }
    return dis;
}













































