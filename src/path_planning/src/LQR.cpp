#include "LQR.h"


LQR::LQR(ros::Publisher pub1, ros::Publisher pub2){
    pub=pub1;
    pub_err=pub2;
    A.resize(Nx,Nx);
    B.resize(Nx,Nu);
    Q=Eigen::MatrixXd::Identity(Nx,Nx)*100;
    R=Eigen::MatrixXd::Identity(Nu,Nu)*5;
    curve_generate();
    referencepath_generate();
    pub.publish(marker);
    init_ego();

}

void LQR::curve_generate(){
    marker.header.frame_id = "path";  // 设置坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "curve_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;  // LINE_STRIP类型表示绘制曲线
    marker.action = visualization_msgs::Marker::ADD;
    // 设置曲线的颜色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // 设置曲线的尺寸
    marker.scale.x = 0.1;  // 线的宽度

    // 添加曲线上的点
    geometry_msgs::Point point;
    for (double t = 0.0; t <= 2 * M_PI; t += 0.005) {
        point.x = (2 * M_PI-t)*10*cos(t);
        point.y = (2 * M_PI-t)*10*sin(t);
        point.z = 0.0;
        marker.points.push_back(point); 
    }
}

void LQR::referencepath_generate(){
    vector<double> x;
    vector<double> y;
    for (int u=0;u<marker.points.size();u++){
        x.push_back(marker.points[marker.points.size()-1-u].x);
        y.push_back(marker.points[marker.points.size()-1-u].y);
    }

    vector<double> dx;
    vector<double> dy;
    dx.push_back(0.0);
    dy.push_back(0.0);
    for (int i=0;i<x.size()-1;i++){
        double xx=x[i+1]-x[i];
        double yy=y[i+1]-y[i];
        dx.push_back(xx);
        dy.push_back(yy);
    }
    vector<double> dxx;
    vector<double> dyy;
    dxx.push_back(0.0);
    dyy.push_back(0.0);
    for (int j=0;j<dx.size()-1;j++){
        double xx=dx[j+1]-dx[j];
        double yy=dy[j+1]-dy[j];
        dxx.push_back(xx);
        dyy.push_back(yy);
    }

    for (int k=0;k<x.size();k++){
        reference_Node node;
        node.x=x[k];
        node.y=y[k];
        node.yaw=atan2(dy[k],dx[k]);
        node.karry=(dyy[k]*dx[k]-dy[k]*dxx[k])/pow(pow(dx[k],2)+pow(dy[k],2),1.5);
        node.v=3.0;
        reference_path.push_back(node);
    }
    reference_path[0].karry=0.0;
}

void LQR::init_ego(){
    ego.header.frame_id = "path";
    ego.header.stamp = ros::Time::now();
    ego.id = 0;
    ego.type = visualization_msgs::Marker::CUBE;
    ego.action = visualization_msgs::Marker::ADD;

    ego.scale.x=2.0;
    ego.scale.y=1.0;
    ego.scale.z=0.6;
    // 设置长方形的位置、姿态和尺寸
    ego_position.x=reference_path[0].x;
    ego_position.y=reference_path[0].y;
    ego_position.yaw=reference_path[0].yaw;
    ego.pose.position.x = ego_position.x;
    ego.pose.position.y = ego_position.y;
    ego.pose.position.z = 0.0;

    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, ego_position.yaw);
    ego.pose.orientation.x = qtn.getX();
    ego.pose.orientation.y = qtn.getY();
    ego.pose.orientation.z = qtn.getZ();
    ego.pose.orientation.w = qtn.getW();

    // 设置长方形的颜色
    ego.color.r = 1.0;
    ego.color.g = 5.0;
    ego.color.b = 0.0;
    ego.color.a = 1.0;  // 完全不透明
    for (int i=0;i<10;i++){
        pub.publish(ego);
        ros::Rate(5).sleep();
        ROS_INFO("============");
    }
}

vector<double> LQR::cal_target_reference(double x,double y){
    double G=10000000000;                               
    for (int i=indexx;i<reference_path.size();i++){
        double dis=sqrt(pow(x-reference_path[i].x,2)+pow(y-reference_path[i].y,2));
        if (G>dis){
            G=dis;
            indexx=i;
        }
    }
    vector<double> target_node={reference_path[indexx].x,reference_path[indexx].y,
                                reference_path[indexx].yaw,reference_path[indexx].v,
                                std::atan(L*reference_path[indexx].karry)};
    return target_node;
}

void LQR::linerization(const double &phi_r, const double &v_r,const double &delta_r){
    A << 1,   0,    -dt*v_r*sin(phi_r),
           0,   1,     dt*v_r*cos(phi_r),
           0,   0,          1;

    B <<  dt*cos(phi_r), 0,
            dt*sin(phi_r), 0,
            dt*tan(delta_r)/L, dt*v_r/(L*pow(cos(delta_r),2));
}

void LQR::update(vector<double> &out){
    double v_out=out[0];
    double throll_out=out[1];
    ROS_INFO("V==%.2f   throll==%.2f",v_out,throll_out);
    //ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    ego_position.x+=v_out*cos(ego_position.yaw)*dt;
    ego_position.y+=v_out*sin(ego_position.yaw)*dt;
    ego_position.yaw+=v_out*tan(throll_out)*dt/L;
    if (ego_position.yaw > M_PI){
        ego_position.yaw -= 2*M_PI;
    }else if (ego_position.yaw < -M_PI){
        ego_position.yaw += 2*M_PI;
    }
    ego.type=visualization_msgs::Marker::DELETE;
    pub.publish(ego);
    ego.type=visualization_msgs::Marker::CUBE;
    ego.action=visualization_msgs::Marker::ADD;
    ego.scale.x=2.0;
    ego.scale.y=1.0;
    ego.scale.z=0.6;
    // 设置长方形的位置、姿态和尺寸
    ego.pose.position.x = ego_position.x;
    ego.pose.position.y = ego_position.y;
    ego.pose.position.z = 0.0;

    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, ego_position.yaw);
    ego.pose.orientation.x = qtn.getX();
    ego.pose.orientation.y = qtn.getY();
    ego.pose.orientation.z = qtn.getZ();
    ego.pose.orientation.w = qtn.getW();

    // 设置长方形的颜色
    ego.color.r = 1.0;
    ego.color.g = 5.0;
    ego.color.b = 0.0;
    ego.color.a = 1.0;  // 完全不透明
    for (int i=0;i<1;i++){
        pub.publish(ego);
        ros::Rate(10).sleep();
        
    }
}

void LQR::solveLQR(){
    
    double ego_x=ego_position.x;
    double ego_y=ego_position.y;
    double ego_yaw=ego_position.yaw;
    vector<double> pose_r =cal_target_reference(ego_x, ego_y);
    double xr=pose_r[0];
    double yr=pose_r[1];
    double yawr=pose_r[2];
    double vr=pose_r[3];
    double throllr=pose_r[4];
    linerization(yawr, vr,throllr);
    double x_err=ego_x-xr;
    double y_err=ego_y-yr;
    double laterr=y_err*cos(yawr)-x_err*sin(yawr);
    std_msgs::Float64 doubleMsg;
    doubleMsg.data=laterr;
    pub_err.publish(doubleMsg);
    Eigen::Vector3d X;
    X << ego_x-xr, ego_y-yr, ego_yaw-yawr;
    if (X(2) > M_PI){
        X(2) -= 2*M_PI;
    }else if (X(2) < -M_PI){
        X(2) += 2*M_PI;
    }
    Eigen::MatrixXd P=claculate_P();
    Eigen::MatrixXd K=-(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
    Eigen::Vector2d OUT=K*X;
    vector<double> output{vr+OUT(0), throllr+OUT(1)};
    update(output);
}

Eigen::MatrixXd LQR::claculate_P(){
    Eigen::MatrixXd Pn=Q;
    Eigen::MatrixXd P_=Q;
    for (int i=0;i<maxiter;i++){
        P_=Q + A.transpose()*Pn*A -A.transpose()*Pn*B*(R+B.transpose()*Pn*B).inverse()
                        *B.transpose()*Pn*A;
        if ((P_-Pn).norm()<0.2) break;
        Pn=P_;
    }
    return P_;
}

void LQR::pub_path(){pub.publish(marker);}















































































