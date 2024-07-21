#include "emplanner.hh"
visualization_msgs::Marker globalpath;
visualization_msgs::Marker localpath;
visualization_msgs::Marker vehicle;
using Vec_f = std::vector<float>;
using Poi_f = std::array<float, 2>;
using Vec_Poi = std::vector<Poi_f>;

PathPlanningNode::PathPlanningNode(){
    pub_marker=nh_.advertise<visualization_msgs::Marker>("visualization_marker",100);
}
PathPlanningNode::~PathPlanningNode() {}
bool PathPlanningNode::init(){
    
    vehicleState_.x=0.0;
    vehicleState_.y=0.0;
    vehicleState_.heading=0.0;
    vehicleState_.yaw=0.0;
    vehicleState_.acceleration=0.0;
    vehicleState_.angular_velocity=0.0;
    vehicleState_.vx=0.0;
    vehicleState_.vy=0.0;
    vehicleState_.velocity=0.0;
    vehicleState_.kappa=0.0;
    odomCallback();
    //加载路网文件
    if (!loadRoadmap(target_speed)) return false;
    GetWayPoints();  //在参考路径的基础上进行踩点,填充wx_ wy_
    
    speedPidControllerPtr_ = std::shared_ptr<PIDController>(
        new PIDController(0.4, 0.1, 0.0));//初始化PID控制器
    lqrController_ = std::shared_ptr<LQRController>(new LQRController());//初始化LQR控制器
    
    // 构建相对平滑的Frenet曲线坐标系，一个中间暂时方案
    csp_obj_ = new Spline2D(wx_, wy_);
    // 全局路径可视化
    PlotGlobalPath();
    UpdateStaticObstacle();

    visTimer_ = nh_.createTimer(ros::Duration(1 / 50),
                              &PathPlanningNode::visTimerLoop,
                              this);  //注册可视化线程
    plannerTimer_ = nh_.createTimer(ros::Duration(1 / plannerFrequency_),
                                  &PathPlanningNode::plannerTimerLoop,
                                  this);  //注册规划线程
    controlTimer_ = nh_.createTimer(ros::Duration(1 / controlFrequency_),
                                  &PathPlanningNode::controlTimerLoop,
                                  this);  //注册控制线程
    
    goalPoint_ = planningPublishedTrajectory_.trajectory_points.back();  //确定目标点
    ROS_INFO("planner node and lqr_control_node init finish!");
    return true;
}

double PathPlanningNode::pid_control(double vr) {
    double ego_speed = vehicleState_.velocity;
    // 位置误差
    double v_err = target_speed - ego_speed;  // 速度误差
    //cout << "v_err: " << v_err << "targetSpeed_ is " << targetSpeed_ << endl;
    double acceleration_cmd =
        speedPidControllerPtr_->Control(v_err, 1 / controlFrequency_);
    return acceleration_cmd;
}

void PathPlanningNode::publishObstacles(){
    if (obstcle_list_.empty()) return;
    // Visualize circular obstacles
    std::size_t idx = 0;
    for (const auto &obs : obstcle_list_) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "CircularObstacles";
        marker.id = idx--;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point point;
        point.x = obs[0];
        point.y = obs[1];
        point.z = 0;
        marker.points.push_back(point);

        const double radius = 1.5;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        pub_marker.publish(marker);
    }
    if (dynamic_obstcle_list_.empty()) return;
    for (const auto &obs : dynamic_obstcle_list_) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "CircularObstacles";
        marker.id = idx--;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point point;
        point.x = obs.x;
        point.y = obs.y;
        point.z = 0;
        marker.points.push_back(point);

        const double radius = 1.5;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        pub_marker.publish(marker);
    }
}

void PathPlanningNode::UpdateStaticObstacle(){
    std::vector<Poi_f> obstcles{{30, 0.0}};
    VehicleState obs1;
    obs1.x=70.0;
    obs1.y=-8.0;
    obs1.heading=M_PI/2;
    obs1.velocity=0.5;
    obs1.acceleration=0.0;
    dynamic_obstcle_list_.push_back(obs1);
    obstcle_list_ = obstcles;
}

void PathPlanningNode::PlotGlobalPath(){
    globalpath.header.frame_id="map";
    globalpath.header.stamp=ros::Time::now();
    globalpath.ns = "globalpath";
    globalpath.id = 0;
    globalpath.type=visualization_msgs::Marker::LINE_STRIP;
    globalpath.action=visualization_msgs::Marker::ADD;
    globalpath.scale.x = 0.2; // 线宽

    // 设置曲线的颜色
    globalpath.color.a = 1.0; // 不透明
    globalpath.color.r = 1.0;
    globalpath.color.g = 0.0;
    globalpath.color.b = 0.0;

    for (int i=0;i<wx_.size();i++){
        geometry_msgs::Point p;
        p.x=wx_[i];
        p.y=wy_[i];
        p.z=0.0;
        globalpath.points.push_back(p);
    }

    Vec_f r_x;
    Vec_f r_y;
    Vec_f ryaw;
    Vec_f rcurvature;
    Vec_f rs;

    global_plan_.poses.clear();
    global_plan_.header.frame_id = "map";
    global_plan_.header.stamp = ros::Time::now();
    // 0.1米的间隔进行踩点
    for (float i = 0; i < csp_obj_->s.back(); i += 0.1) {
        std::array<float, 2> point_ = csp_obj_->calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj_->calc_yaw(i));
        rcurvature.push_back(csp_obj_->calc_curvature(i));
        rs.push_back(i);

        geometry_msgs::PoseStamped pt;
        pt.header.stamp = ros::Time::now();
        pt.header.frame_id = "map";
        pt.pose.position.x = point_[0];
        pt.pose.position.y = point_[1];
        pt.pose.position.z = i;  //使用position.z存储路径的s
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(csp_obj_->calc_yaw(i));
        global_plan_.poses.push_back(pt);
    }

    end_x_ = r_x.back();
    end_y_ = r_y.back();
    end_s_ = rs.back();
    ROS_INFO_STREAM("s_end= " << end_s_);

}
void PathPlanningNode::visTimerLoop(const ros::TimerEvent &){
    pub_marker.publish(globalpath);
    pub_marker.publish(localpath);
    for (const auto &obs : obstcle_list_) {
        publishObstacles();
    }
    publishVehicle();
}

void PathPlanningNode::odomCallback(){
    if (firstRecord_) {
        vehicleState_.planning_init_x = vehicleState_.x;
        vehicleState_.planning_init_y = vehicleState_.y;
        firstRecord_ = false;
    }
    vehicleState_.heading = vehicleState_.yaw;
    vehicleState_.velocity =  // 速度
      std::sqrt(vehicleState_.vx * vehicleState_.vx +
                vehicleState_.vy * vehicleState_.vy);
}

void PathPlanningNode::UpdateDynamicObs(){
    if (dynamic_obstcle_list_.empty()) return;
    for (int i=0;i<dynamic_obstcle_list_.size();i++){
        double vx=dynamic_obstcle_list_[i].velocity*cos(dynamic_obstcle_list_[i].heading);
        double vy=dynamic_obstcle_list_[i].velocity*sin(dynamic_obstcle_list_[i].heading);
        double ax=dynamic_obstcle_list_[i].acceleration*cos(dynamic_obstcle_list_[i].heading);
        double ay=dynamic_obstcle_list_[i].acceleration*sin(dynamic_obstcle_list_[i].heading);
        dynamic_obstcle_list_[i].x += vx*0.01 + 0.5*ax*0.0001;
        dynamic_obstcle_list_[i].y += vy*0.01 + 0.5*ay*0.0001;
    }
}

void PathPlanningNode::plannerTimerLoop(const ros::TimerEvent &) {
    if (!firstRecord_ && !isReachGoal_) {  //有定位数据开始规划
        // TODO:这里之后可以再被优化，采用更好的Frenet坐标系取点方式。
        
        const double ego_s = GetNearestReferenceLength(vehicleState_);

        const double ego_l = GetNearestReferenceLatDist(vehicleState_);
        const double ego_speed = vehicleState_.velocity;
        // std::cout<<"ego_s=="<<ego_s<<std::endl;
        //std::cout<<"ego_l=="<<ego_l<<std::endl;
        // std::cout<<"ego_speed=="<<ego_speed<<std::endl;
        s0_ = ego_s;
        if (std::abs(ego_speed) > 1e-3) {
        c_speed_ = ego_speed;
        }
        c_d_ = ego_l;

        // Idea:
        // 判断是否是终点,这里之后需要优化一下，加一个精准停车功能，然后缩小误差范围，发送Stop命令
        if (std::abs(s0_ - end_s_) < 1.0) {
            // break;
            isReachGoal_ = true;
            ROS_INFO("Goal Reached!");
        }

        FrenetOptimalTrajectory frenet_optimal_trajectory;
        // to-do step 1 finish frenet_optimal_planning
        FrenetPath final_path = frenet_optimal_trajectory.frenet_optimal_planning(
            *csp_obj_, s0_, c_speed_, c_d_, c_d_d_, c_d_dd_, 
                obstcle_list_,dynamic_obstcle_list_);
        if (final_path.s.empty()){
            c_d_ = last_final_path.d.back();
            c_d_d_ = last_final_path.d_d.back();
            c_d_dd_ = last_final_path.d_dd.back();
        }
        if (!final_path.s.empty() && !near_goal_) {
            s0_ = final_path.s[1];
            c_d_ = final_path.d[1];
            c_d_d_ = final_path.d_d[1];
            c_d_dd_ = final_path.d_dd[1];
            c_speed_ = final_path.s_d[1];

            // 可视化
            publishLocalPlan(final_path);
            const auto trajectory = GetTrajectoryFormFrenetPath(final_path);
            planningPublishedTrajectoryDebug_ = trajectory;
            last_trajectory_ = trajectory;

            if (std::abs(final_path.s.back() - end_s_) < 1.0) {
                ROS_INFO("Near Goal");
                near_goal_ = true;
            }
        } else {
            // Backup
            planningPublishedTrajectoryDebug_ = last_trajectory_;
        }
        last_final_path=final_path;
        plannerFlag_ = true;
    }
    std::cout<<"=========================================="<<std::endl;
}

void PathPlanningNode::controlTimerLoop(const ros::TimerEvent &){
    ControlCmd cmd;
    if (plannerFlag_) {  //有定位数据开始控制
        //小于容忍距离，车辆速度设置为0
        if (pointDistance(goalPoint_, vehicleState_.x, vehicleState_.y) < goalTolerance_) {
            targetSpeed_ = 0;
            ROS_INFO("Goal Readched");
            isReachGoal_ = true;
        }
        double vr;
        if (!isReachGoal_) {
            double min_dis=numeric_limits<double>::max();
            double xr,yr,thetar,kr;
            // double vx=vehicleState_.velocity*cos(vehicleState_.heading);
            // double vy=vehicleState_.velocity*sin(vehicleState_.heading);
            // double ax=vehicleState_.acceleration*cos(vehicleState_.heading);
            // double ay=vehicleState_.acceleration*cos(vehicleState_.heading);
            // double xx=vehicleState_.x + vx*0.01+0.5*ax*0.0001;
            // double yy=vehicleState_.y + vy*0.01+0.5*ay*0.0001;
            for (const auto point : planningPublishedTrajectoryDebug_.trajectory_points){
                double dis=pow(point.x-vehicleState_.x,2) + 
                            pow(point.y-vehicleState_.y,2);
                if (dis<min_dis){
                    min_dis=dis;
                    xr=point.x;
                    yr=point.y;
                    thetar=point.heading;
                    kr=point.kappa;
                    vr=point.v;
                }
            }
            lqrController_->Init_Parameters(xr,yr,thetar,vr,kr);
            auto out=lqrController_->solveLQR(vehicleState_.x, vehicleState_.y,
                    vehicleState_.heading);
            cmd.steer_target=std::get<1>(out);
        }
        cmd.acc = pid_control(vr);
        if (cmd.steer_target > M_PI/4) cmd.steer_target=M_PI/4;
        if (cmd.steer_target < -M_PI/4) cmd.steer_target=-M_PI/4;
        
        if (isReachGoal_){
            cmd.acc=-vehicleState_.velocity;
            cmd.steer_target=0.0;
        }
        if (cmd.acc>6.0) cmd.acc=6.0;
        if (cmd.acc<-4.0) cmd.acc=-4.0;
        //std::cout<<"acc=="<<cmd.acc<<"  steer=="<<cmd.steer_target<<std::endl;
        std::cout<<"vr=="<<vr<<"  v=="<<vehicleState_.velocity<<std::endl;
        UpdateVehicle(cmd.acc,cmd.steer_target);
        
        UpdateDynamicObs();
        
        
    }
}




void PathPlanningNode::UpdateVehicle(const double acc, const double steer){
    double vx=vehicleState_.velocity*cos(vehicleState_.heading);
    double vy=vehicleState_.velocity*sin(vehicleState_.heading);
    double ax=acc*cos(vehicleState_.heading);
    double ay=acc*sin(vehicleState_.heading);
    vehicleState_.acceleration=acc;
    vehicleState_.x += vx*0.01 + 0.5*ax*0.0001;
    vehicleState_.y += vy*0.01 + 0.5*ay*0.0001;
    vehicleState_.velocity += acc*0.01;
    vehicleState_.heading+=vehicleState_.velocity*tan(steer)*0.01/3.0;
    
}

void PathPlanningNode::publishVehicle(){
    vehicle.header.frame_id="map";
    vehicle.header.stamp=ros::Time::now();
    vehicle.ns = "vehicle";
    vehicle.id = -100;
    vehicle.type=visualization_msgs::Marker::CUBE;
    vehicle.scale.x = 1.0;
    vehicle.scale.y = 0.5;
    vehicle.scale.z = 0.2;
    vehicle.pose.position.x=vehicleState_.x;
    vehicle.pose.position.y=vehicleState_.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, vehicleState_.heading); // 设置为绕 z 轴旋转 90 度
    vehicle.pose.orientation.w=q.getW();
    vehicle.pose.orientation.x=q.getX();
    vehicle.pose.orientation.y=q.getY();
    vehicle.pose.orientation.z=q.getZ();
    // 设置曲线的颜色
    vehicle.color.a = 1.0; // 不透明
    vehicle.color.r = 1.0;
    vehicle.color.g = 5.0;
    vehicle.color.b = 0.0;
    
    pub_marker.publish(vehicle);
}

TrajectoryData PathPlanningNode::GetTrajectoryFormFrenetPath(const FrenetPath &path){
    TrajectoryData trajectory;
    const int traj_size = path.t.size();
    trajectory.trajectory_points.reserve(traj_size);
    for (int i = 0; i < traj_size; i++) {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = path.x[i];
        trajectory_pt.y = path.y[i];
        trajectory_pt.v = path.ds[i];
        trajectory_pt.a = 0.0;
        trajectory_pt.heading = path.yaw[i];
        trajectory_pt.kappa = path.c[i];
        trajectory.trajectory_points.push_back(trajectory_pt);
    }
    return trajectory;
}

void PathPlanningNode::publishLocalPlan(const FrenetPath &final_path){
    localpath.header.frame_id="map";
    localpath.header.stamp=ros::Time::now();
    localpath.ns = "localpath";
    localpath.id = 1;
    localpath.type=visualization_msgs::Marker::LINE_STRIP;
    localpath.action=visualization_msgs::Marker::ADD;
    localpath.scale.x = 0.1; // 线宽

    // 设置曲线的颜色
    localpath.color.a = 1.0; // 不透明
    localpath.color.r = 0.0;
    localpath.color.g = 1.0;
    localpath.color.b = 0.0;

    for (int i=0;i<final_path.x.size();i++){
        geometry_msgs::Point p;
        p.x=final_path.x[i];
        p.y=final_path.y[i];
        p.z=0.0;
        localpath.points.push_back(p);
    }
}

void PathPlanningNode::GetWayPoints() {
    const int refline_size =
        planningPublishedTrajectory_.trajectory_points.size();

    const auto &trajectory_pt = planningPublishedTrajectory_.trajectory_points;

    double sum_s = 0;
    wx_.push_back(trajectory_pt[0].x);
    wy_.push_back(trajectory_pt[0].y);

    for (int i = 1; i < refline_size; i++) {
        const double dx = trajectory_pt[i].x - trajectory_pt[i - 1].x;
        const double dy = trajectory_pt[i].y - trajectory_pt[i - 1].y;
        const double s = std::sqrt(dx * dx + dy * dy);
        sum_s += s;

        // 每隔2米的距离进行踩点
        if (sum_s > 2.0) {
        wx_.push_back(trajectory_pt[i].x);
        wy_.push_back(trajectory_pt[i].y);
        sum_s = 0;
        }
    }

}

bool PathPlanningNode::loadRoadmap(const double target_speed) {

  std::vector<std::pair<double, double>> xy_points;
  double ii=0.0;
  while (ii<2000.0) {
    double pt_x = ii/10.0;
    double pt_y = 0.0;
    xy_points.push_back(std::make_pair(pt_x, pt_y));
    ii+=2.0;
  }
  std::vector<double> headings, accumulated_s, kappas, dkappas;
  //根据离散的点组成的路径，生成路网航向角,累计距离，曲率，曲率的导数
  std::unique_ptr<ReferenceLine> reference_line =
      std::make_unique<ReferenceLine>(xy_points);
  reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas,
                                     &dkappas);

  for (size_t i = 0; i < headings.size(); i++) {
    TrajectoryPoint trajectory_pt;
    trajectory_pt.x = xy_points[i].first;
    trajectory_pt.y = xy_points[i].second;
    trajectory_pt.heading = headings[i];
    trajectory_pt.kappa = kappas[i];
    planningPublishedTrajectory_.trajectory_points.push_back(trajectory_pt);
  }
  return true;
}

int PathPlanningNode::GetNearestReferenceIndex(const VehicleState &ego_state) {
    double min_dist = std::numeric_limits<double>::max();
    size_t min_index = 0;

    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
        const double distance =
            DistanceXY(ego_state, global_plan_.poses[i].pose.position);
        if (distance < min_dist) {
        min_dist = distance;
        min_index = i;
        }
    }
    return min_index;
}

double PathPlanningNode::GetNearestReferenceLength(const VehicleState &ego_state) {
    return global_plan_.poses[GetNearestReferenceIndex(ego_state)]
    .pose.position.z;  // s存在position.z中
}

double PathPlanningNode::GetNearestReferenceLatDist(const VehicleState &ego_state) {
    double min_dist = std::numeric_limits<double>::max();
    size_t min_index = 0;

    for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i) {
        const double distance =
            DistanceXY(ego_state, global_plan_.poses[i].pose.position);
        if (distance < min_dist) {
        min_dist = distance;
        min_index = i;
        }
    }
    const int sign = LeftOfLine(ego_state, global_plan_.poses[min_index],
                                global_plan_.poses[min_index + 1])
                        ? 1
                        : -1;
    //std::cout<<"         sign=="<<sign<<std::endl;
    return sign * min_dist;
}

bool PathPlanningNode::LeftOfLine(const VehicleState &p,
                                  const geometry_msgs::PoseStamped &p1,
                                  const geometry_msgs::PoseStamped &p2) {
    const double tmpx = (p1.pose.position.x - p2.pose.position.x) /
                            (p1.pose.position.y - p2.pose.position.y) *
                            (p.y - p2.pose.position.y) +
                            p2.pose.position.x;
    double tor1[2]={p2.pose.position.x-p1.pose.position.x , 
                    p2.pose.position.y-p1.pose.position.y};
    double tor2[2]={p.x-p1.pose.position.x , p.y-p1.pose.position.y};
    double flag=tor1[0]*tor2[1] - tor1[1]*tor2[0];
    // if (tmpx > p.x)  //当tmpx>p.x的时候，说明点在线的左边，小于在右边，等于则在线上。
    //     return true;
    if (flag > 0) return true;
    return false;
}