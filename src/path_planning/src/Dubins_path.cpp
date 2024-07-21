#include "Dubins_path.h"


double pi_2_pi(double theta){
    while (theta > M_PI){
        theta -= 2*M_PI;
    }
    while(theta < -M_PI){
        theta += 2*M_PI;
    }
    return theta;
}

double mod2pi(double theta){
    return theta-2.0*M_PI*floor(theta/M_PI/2.0);
}

Trajectory LSL(double alpha, double beta, double dist){
    double sin_a=sin(alpha);
    double sin_b=sin(beta);
    double cos_a=cos(alpha);
    double cos_b=cos(beta);
    double cos_a_b=cos(alpha-beta);
    double p_lsl=2 + pow(dist,2) - 2*cos_a_b + 2*dist*(sin_a-sin_b);
    Trajectory LSL_tracjory;
    LSL_tracjory.mode="LSL";
    if (p_lsl<0){
        return LSL_tracjory;
    }else{
        p_lsl=sqrt(p_lsl);
    }
    double denominate=dist + sin_a - sin_b;
    double t_lsl=mod2pi(-alpha + atan2(cos_b-cos_a , denominate));
    double q_lsl=mod2pi(beta - atan2(cos_b-cos_a , denominate));
    LSL_tracjory.t=t_lsl;
    LSL_tracjory.p=p_lsl;
    LSL_tracjory.q=q_lsl;

    return LSL_tracjory;

}

Trajectory RSR(double alpha, double beta, double dist){
    double sin_a=sin(alpha);
    double sin_b=sin(beta);
    double cos_a=cos(alpha);
    double cos_b=cos(beta);
    double cos_a_b=cos(alpha-beta);
    double p_rsr=2 + pow(dist,2) - 2*cos_a_b + 2*dist*(sin_b-sin_a);
    Trajectory RSR_trajectory;
    RSR_trajectory.mode="RSR";
    if (p_rsr<0){
        return RSR_trajectory;
    }else{
        p_rsr=sqrt(p_rsr);
    }
    double denominate= dist - sin_a + sin_b;
    double t_rsr=mod2pi(alpha - atan2(cos_a-cos_b , denominate));
    double q_rsr=mod2pi(-beta + atan2(cos_a-cos_b , denominate));
    RSR_trajectory.t=t_rsr;
    RSR_trajectory.q=q_rsr;
    RSR_trajectory.p=p_rsr;
    // ROS_INFO("t=%.3f",t_rsr);
    // ROS_INFO("p=%.3f",p_rsr);
    // ROS_INFO("q=%.3f",q_rsr);
    return RSR_trajectory;
    
}

Trajectory LSR(double alpha, double beta, double dist){
    double sin_a=sin(alpha);
    double sin_b=sin(beta);
    double cos_a=cos(alpha);
    double cos_b=cos(beta);
    double cos_a_b=cos(alpha-beta);
    double p_lsr=-2 + pow(dist,2) + 2*cos_a_b + 2*dist*(sin_a+sin_b);
    Trajectory LSR_trajectory;
    LSR_trajectory.mode="LSR";
    if (p_lsr<0){
        return LSR_trajectory;
    }else{
        p_lsr=sqrt(p_lsr);
    }
    double rec=atan2(-cos_a-cos_b , dist+sin_a+sin_b) - atan2(-2.0 , p_lsr);
    double t_lsr=mod2pi(-alpha + rec);
    double q_lsr=mod2pi(-mod2pi(beta) + rec);
    LSR_trajectory.t=t_lsr;
    LSR_trajectory.p=p_lsr;
    LSR_trajectory.q=q_lsr;
    return LSR_trajectory;
}

Trajectory RSL(double alpha, double beta, double dist){
    double sin_a=sin(alpha);
    double sin_b=sin(beta);
    double cos_a=cos(alpha);
    double cos_b=cos(beta);
    double cos_a_b=cos(alpha - beta);
    double p_rsl=-2 + pow(dist,2) + 2*cos_a_b - 2*dist*(sin_a+sin_b);
    Trajectory RSL_trajectory;
    RSL_trajectory.mode="RSL";
    if (p_rsl<0){
        return RSL_trajectory;
    }else{
        p_rsl=sqrt(p_rsl);
    }
    double rec=atan2(cos_a+cos_b , dist-sin_a-sin_b) - atan2(2.0 , p_rsl);
    double t_rsl=mod2pi(alpha - rec);
    double q_rsl=mod2pi(beta - rec);
    RSL_trajectory.t=t_rsl;
    RSL_trajectory.p=p_rsl;
    RSL_trajectory.q=q_rsl;
    return RSL_trajectory;
}

Trajectory RLR(double alpha, double beta, double dist){
    double sin_a=sin(alpha);
    double sin_b=sin(beta);
    double cos_a=cos(alpha);
    double cos_b=cos(beta);
    double cos_a_b=cos(alpha - beta);
    double rec=(6.0 - pow(dist,2) + 2.0*cos_a_b + 2.0*dist*(sin_a-sin_b))/8.0;
    Trajectory RLR_trajectory;
    RLR_trajectory.mode="RLR";
    if (abs(rec)>1.0) return RLR_trajectory;

    double p_rlr=mod2pi(2*M_PI - acos(rec));
    double t_rlr=mod2pi(alpha - atan2(cos_a - cos_b , dist-sin_a + sin_b) + mod2pi(p_rlr/2.0));
    double q_rlr=mod2pi(alpha - beta -t_rlr + mod2pi(p_rlr));
    RLR_trajectory.t=t_rlr;
    RLR_trajectory.p=p_rlr;
    RLR_trajectory.q=q_rlr;
    return RLR_trajectory;
    
}

Trajectory LRL(double alpha, double beta, double dist){
    double sin_a=sin(alpha);
    double sin_b=sin(beta);
    double cos_a=cos(alpha);
    double cos_b=cos(beta);
    double cos_a_b=cos(alpha - beta);
    double rec=(6.0 - pow(dist,2) + 2.0*cos_a_b + 2.0*dist*(sin_b - sin_a))/8.0;
    Trajectory LRL_trajectory;
    LRL_trajectory.mode="LRL";
    if (abs(rec)>1.0) return LRL_trajectory;
    
    double p_lrl=mod2pi(2*M_PI - acos(rec));
    double t_lrl=mod2pi(-alpha - atan2(cos_a - cos_b , dist+sin_a-sin_b) + p_lrl/2.0);
    double q_lrl=mod2pi(mod2pi(beta) - alpha - t_lrl + mod2pi(p_lrl));
    LRL_trajectory.t=t_lrl;
    LRL_trajectory.p=p_lrl;
    LRL_trajectory.q=q_lrl;
    return LRL_trajectory;
}

void interpolate(int ind,double l,char m,double maxc,double ox,
                    double oy,double oyaw,vector<double> &px,vector<double> &py,
                    vector<double> &pyaw){
    if (m=='S'){
        px[ind]=ox + l/maxc * cos(oyaw);
        py[ind]=oy + l/maxc * sin(oyaw);
        pyaw[ind]=oyaw;
    }else{
        double ldx=sin(l)/maxc;
        double ldy;
        if (m=='L'){
            ldy=(1.0 - cos(l))/maxc; 
        }else if (m=='R'){
            ldy=(1.0 - cos(l))/(-maxc);
        }
        double gdx=cos(-oyaw)*ldx + sin(-oyaw)*ldy;
        double gdy=-sin(-oyaw)*ldx + cos(-oyaw)*ldy;
        px[ind]=ox + gdx;
        py[ind]=oy + gdy;
    }
    if (m=='L') pyaw[ind]=oyaw + l;
    if (m=='R') pyaw[ind]=oyaw - l;

}

Best_trajectory generate_local_course(double L, vector<double> lengths,
                string mo, double maxc, double step_size){
    int point_num=int(L/step_size) + lengths.size() + 3;
    vector<char> mode={mo[0], mo[1], mo[2]};
    vector<int> len_mode={0 , 1 , 2};
    vector<double> px(point_num , 0.0);
    vector<double> py(point_num , 0.0);
    vector<double> pyaw(point_num , 0.0);

    int ind=1;
    double d;

    double ll=0;
    for (int i=0; i<mode.size(); i++){
        char m=mode[i];
        //std::cout<<m<<std::endl;
        double l=lengths[i];
        if (l > 0.0){
            d=step_size;
        }else{
            d=-step_size;
        }
        
        double ox=px[ind];
        double oy=py[ind];
        double oyaw=pyaw[ind];
        //std::cout<<ox<<std::endl;
        ind-=1;
        double pd;
        if (i>=1 && (lengths[i-1]*lengths[i])>0){
            pd=-d-ll;
        }else{
            pd=d-ll;
        }
        while(abs(pd) <= abs(l)){
            ind+=1;
            interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw);
            pd+=d;
        }
        ll=l-pd-d;
        ind+=1;
        interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw);
    }
    Best_trajectory best_tra;
    if (px.size() <= 1) return best_tra;
    while(px.size()>=1 && px.back()==0.0){
        px.pop_back();
        py.pop_back();
        pyaw.pop_back();

    }

    best_tra.x_list=px;
    best_tra.y_list=py;
    best_tra.yaw_list=pyaw;
    
    return best_tra;
}

/*
std::vector<std::string> mode = {"LSL", "RSR", "LSR"};
    std::vector<double> lengths = {1.0, 2.0, 3.0};

    // 确保 mode 和 lengths 具有相同的大小
    if (mode.size() != lengths.size()) {
        std::cerr << "Error: mode and lengths must have the same size." << std::endl;
        return 1;  // 返回错误代码
    }

    // 使用 std::tie 和 std::tuple 创建元组
    for (const auto& [m, l, i] : zip(mode, lengths)) {
        std::cout << "Mode: " << m << ", Length: " << l << ", Index: " << i << std::endl;
    }
*/

Best_trajectory planning_from_origin(double gx, double gy, double gyaw,
                                     double curv, double step_size){
    double D=sqrt(pow(gx,2)+pow(gy,2));
    double d=D*curv;
    double theta=mod2pi(atan2(gy,gx));
    double alpha=mod2pi(-theta);
    double beta=mod2pi(gyaw-theta);
    double best_cost=std::numeric_limits<double>::max();
    double bt=std::numeric_limits<double>::max();
    double bp=std::numeric_limits<double>::max();
    double bq=std::numeric_limits<double>::max();
    string best_mode;
    // 定义一个函数指针类型
    using FunctionPointer = Trajectory(*)(double , double , double);
    // 创建函数指针数组
    FunctionPointer functions[] = {LSL, RSR, LSR, RSL, RLR, LRL};
    
    for (FunctionPointer func : functions){
        Trajectory tra=func(alpha, beta, d);
        
        if (isnan(tra.t)) continue;
        double cost=abs(tra.t) + abs(tra.p) + abs(tra.q);
        if (best_cost > cost){
            bt=tra.t;
            bp=tra.p;
            bq=tra.q;
            best_mode=tra.mode;
            best_cost=cost;
        } 
    }
    vector<double> lengths={bt, bp, bq};
    double sum_=bt+bp+bq;
    Best_trajectory best_tra=generate_local_course(sum_, lengths,
                best_mode, curv, step_size);
    best_tra.best_cost=best_cost;
    best_tra.best_mode=best_mode;
    ROS_INFO("%s",best_mode.c_str());
    
    return best_tra;
}

PATH calc_dubins_path(double sx, double sy, double syaw, double gx, double gy, double gyaw,
                        double curv, double step_size){
    gx=gx-sx;
    gy=gy-sy;
    // 创建旋转矩阵
    Eigen::Matrix3d rotation_matrix = 
                    Eigen::AngleAxisd(syaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // 提取旋转矩阵的前两行前两列
    Eigen::Matrix2d l_rot = rotation_matrix.block<2, 2>(0, 0);
    Eigen::Array2d le_xy = Eigen::Vector2d(gx,gy).transpose()*(l_rot);
    double le_yaw=gyaw-syaw;
    
    Best_trajectory lp_trajectory = planning_from_origin(le_xy[0], le_xy[1], le_yaw,
                                     curv, step_size);
    
    Eigen::Matrix3d rerotation_matrix = 
                    Eigen::AngleAxisd(-syaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // 提取旋转矩阵的前两行前两列
    Eigen::Matrix2d rot = rerotation_matrix.block<2, 2>(0, 0);
    //Eigen::Map<Eigen::VectorXd> eigenVector(myVector.data(), myVector.size());
    Eigen::Map<Eigen::VectorXd> lp_x(lp_trajectory.x_list.data(), lp_trajectory.x_list.size());
    Eigen::Map<Eigen::VectorXd> lp_y(lp_trajectory.y_list.data(), lp_trajectory.y_list.size());
    Eigen::MatrixXd lp_xy(lp_x.size(),2);
    lp_xy << lp_x,lp_y;
    Eigen::MatrixXd converted_xy=lp_xy*(rot);
    //Eigen::VectorXd vectorFromCol = matrix.col(0);
    Eigen::VectorXd x_=converted_xy.col(0);
    Eigen::VectorXd y_=converted_xy.col(1);
    

    std::cout<<x_.size();

    //std::vector<double> stdVector(eigenVector.data(), eigenVector.data() + eigenVector.size());
    vector<double> x_list(x_.data(), x_.data() + x_.size());
    vector<double> y_list(y_.data(), y_.data() + y_.size());
    vector<double> yaw_list;
    for (double &element : x_list){
        element += sx;
    }
    for (double &element : y_list){
        element += sy;
    }
    for (int i=0; i<lp_trajectory.yaw_list.size(); i++){
        double pip=pi_2_pi(lp_trajectory.yaw_list[i]+syaw);
        yaw_list.push_back(pip);
    }
    double lengths=lp_trajectory.best_cost;
    string mode=lp_trajectory.best_mode;

    return PATH(lengths, mode, x_list, y_list, yaw_list);



}

double deg2rad(double degrees){
    return degrees * M_PI / 180.0;
}

Dubins_Path generate_path(vector<vector<double>> &states){
    double max_c=0.25;
    Dubins_Path path;
    
    
    double s_x=states[0][0];
    double s_y=states[0][1];
    double s_yaw=states[0][2];

    double g_x=states[1][0];
    double g_y=states[1][1];
    double g_yaw=states[1][2];

    PATH path_i=calc_dubins_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, max_c);
    
    //vector1.insert(vector1.end(), vector2.begin(), vector2.end());
    path.x.insert(path.x.end(), path_i.x_list.begin(), path_i.x_list.end());
    path.y.insert(path.y.end(), path_i.y_list.begin(), path_i.y_list.end());
    path.yaw.insert(path.yaw.end(), path_i.yaw_list.begin(), path_i.yaw_list.end());
    
    return path;
    
}

void visualization_path(Dubins_Path &path, ros::Publisher &pub_marker){
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id="path";
    line_marker.header.stamp=ros::Time::now();
    line_marker.id=0;
    line_marker.type=visualization_msgs::Marker::LINE_STRIP;
    line_marker.action=visualization_msgs::Marker::ADD;
    line_marker.scale.x=0.05;
    line_marker.color.r=1.0;
    line_marker.color.g=5.0;
    line_marker.color.b=0.0;
    line_marker.color.a=1.0;
    for (int i=0;i<path.x.size();i++){
        geometry_msgs::Point point;
        point.x=path.x[i];
        point.y=path.y[i];
        point.z=0.0;
        line_marker.points.push_back(point);
    }
    ros::Rate(2).sleep();
    pub_marker.publish(line_marker);

}


