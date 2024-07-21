#include"MPC.h"

Eigen::MatrixXd MPC::qiumi(Eigen::MatrixXd A,int u){
    Eigen::MatrixXd P=A;
    if (u==0) return P.Identity(Nx+Nu,Nx+Nu);
    if (u==1) return P;
    for (int i=1;i<u;i++){
        P=P*A;
    }
    return P;
}

MPC::MPC(ros::Publisher pub1, ros::Publisher pub2){
    Qq=Eigen::MatrixXd::Identity(Nx*Np,Nx*Np)*100;
    Rr=Eigen::MatrixXd::Identity(Nc*Nu,Nc*Nu);
    Ad_.resize(Nx,Nx);
    Bd_.resize(Nx,Nu);
    kesi.resize(Nx+Nu);
    pub=pub1;
    pub_err=pub2;
    curve_generate();
    referencepath_generate();
    pub.publish(marker);
    MPC::init_ego();
}
void MPC::pub_path() {pub.publish(marker);}
void MPC::curve_generate(){

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

void MPC::referencepath_generate(){
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

void MPC::init_ego(){
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

vector<double> MPC::cal_target_reference(double x,double y){
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
void MPC::linerization(const double &phi_r, const double &v_r,const double &delta_r){
    Ad_ << 1,   0,    -dt*v_r*sin(phi_r),
           0,   1,     dt*v_r*cos(phi_r),
           0,   0,          1;

    Bd_ <<  dt*cos(phi_r), 0,
            dt*sin(phi_r), 0,
            dt*tan(delta_r)/L, dt*v_r/(L*pow(cos(delta_r),2));
}
void MPC::update(vector<double> &out){
    double v_out=out[0];
    double throll_out=out[1];
    
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
void MPC::solveQP(){
    ROS_INFO("------------------------------------");
    double ego_x=ego_position.x;
    double ego_y=ego_position.y;
    double ego_yaw=ego_position.yaw;
    vector<double> pose_r =cal_target_reference(ego_x, ego_y);
    double xr=pose_r[0];
    double yr=pose_r[1];
    double yawr=pose_r[2];
    double vr=pose_r[3];
    double throllr=pose_r[4];
    double x_err=ego_x-xr;
    double y_err=ego_y-yr;
    double laterr=y_err*cos(yawr)-x_err*sin(yawr);
    std_msgs::Float64 doubleMsg;
    doubleMsg.data=laterr;
    pub_err.publish(doubleMsg);

    Eigen::Vector3d X_real,X_r;
    X_real << ego_x, ego_y, ego_yaw;
    X_r << xr, yr, yawr;
    linerization(yawr, vr,throllr);
    Eigen::Vector2d U;
    U << uk_1[0], uk_1[1];
    // kesi.block(0,0,Nx,1)=X_real-X_r;
    // kesi.block(3,0,Nu,1)=U;
    kesi.resize(Nx+Nu);
    kesi << ego_x-xr, ego_y-yr, ego_yaw-yawr, uk_1[0], uk_1[1];
    if (kesi(2)>M_PI){
        kesi(2) -= 2*M_PI;
    }else if(kesi(2)<-M_PI){
        kesi(2) += 2*M_PI;
    }
    Eigen::MatrixXd A(Nx+Nu,Nx+Nu);
    Eigen::MatrixXd B(Nx+Nu,Nu);
    A.block(0,0,Nx,Nx)=Ad_;
    A.block(0,3,Nx,Nu)=Bd_;
    A.block(3,3,Nu,Nu)=Eigen::MatrixXd::Identity(Nu,Nu);
    A.block(3,0,Nu,Nx)=Eigen::MatrixXd::Zero(Nu,Nx);
    B.block(0,0,Nx,Nu)=Bd_;
    B.block(3,0,Nu,Nu)=Eigen::MatrixXd::Identity(Nu,Nu);
    // for (int i=0;i<A.rows();i++){
    //     for (int j=0;j<A.cols();j++){
    //         cout<<A(i,j);
    //     }
    //     cout<<endl;
    // }
    Eigen::MatrixXd C(Nx,Nx+Nu);
    C.block(0,0,Nx,Nx)=Eigen::MatrixXd::Identity(Nx,Nx);
    C.block(0,3,Nx,Nu)=Eigen::MatrixXd::Zero(Nx,Nu);

    Eigen::MatrixXd PHI(Nx*Np,Nx+Nu);
    for (int i=0;i<Np;i++){
        PHI.block(i*Nx,0,Nx,Nx+Nu)=C*qiumi(A,i+1);
    }

    Eigen::MatrixXd THETA(Nx*Np,Nc*Nu);
    for (int i=0;i<Np;i++){
        for (int j=0;j<Nc;j++){
            if (i>=j){
                THETA.block(i*Nx,j*Nu,Nx,Nu)=C*qiumi(A,i-j)*B;
            }else{
                THETA.block(i*Nx,j*Nu,Nx,Nu)=Eigen::MatrixXd::Zero(Nx,Nu);
            }
        }
    }
    // for (int i=0;i<THETA.rows();i++){
    //     for (int j=0;j<THETA.cols();j++){
    //         cout<<THETA(i,j);
    //     }
    //     cout<<endl;
    // }
    
    Eigen::MatrixXd H(Nc*Nu+1,Nc*Nu+1);
    H.setZero();
    H.block(0,0,Nc*Nu,Nc*Nu)=THETA.transpose()*Qq*THETA+Rr;
    H(Nc*Nu,Nc*Nu)=row;
    
    Eigen::MatrixXd E=PHI*kesi;

    Eigen::MatrixXd g(Nc*Nu+1,1);
    g.block(0,0,Nc*Nu,1)=(E.transpose()*Qq*THETA).transpose();
    g(Nc*Nu,0)=0;
    
    Eigen::MatrixXd A_t=Eigen::MatrixXd::Identity(Nu,Nu);
    Eigen::MatrixXd A_I=Eigen::MatrixXd::Zero(Nc*Nu,Nc*Nu);
    for (int i=0;i<Nc;i++){
        for (int j=0;j<Nc;j++){
            if (i>=j){
                A_I.block(Nu*i,Nu*j,Nu,Nu)=A_t;
            }
        }
    }

    Eigen::MatrixXd Ut=Eigen::MatrixXd::Zero(Nc*Nu,1);
    for (int i=0;i<Nc;i++){
        Ut.block(i*Nu,0,Nu,1)=U;
    }
    
    Eigen::MatrixXd A_cons=Eigen::MatrixXd::Zero(2*Nc*Nu,Nc*Nu+1);
    A_cons.block(0,0,Nc*Nu,Nc*Nu)=A_I;
    A_cons.block(Nc*Nu,0,Nc*Nu,Nc*Nu)=Eigen::MatrixXd::Identity(Nc*Nu,Nc*Nu);

    Eigen::MatrixXd lb=Eigen::MatrixXd::Zero(Nc*Nu*2,1);
    Eigen::MatrixXd lu=Eigen::MatrixXd::Zero(Nc*Nu*2,1);
    Eigen::MatrixXd Umax(Nc*Nu,1);
    Eigen::MatrixXd Umax_(Nu,1);
    Umax_ << umax_v, umax_throll;
    Eigen::MatrixXd Umin(Nc*Nu,1);
    Eigen::MatrixXd Umin_(Nu,1);
    Umin_ << umin_v, umin_throll;
    
    Eigen::MatrixXd deltaUmin(Nc*Nu,1);
    Eigen::MatrixXd deltaUmax(Nc*Nu,1);
    Eigen::MatrixXd deltaUmax_(Nu,1);
    Eigen::MatrixXd deltaUmin_(Nu,1);
    deltaUmax_ << delta_umax_v, delta_umax_throll;
    deltaUmin_ << delta_umin_v, delta_umin_throll;

    for (int i=0;i<Nc;i++){
        Umax.block(i*Nu,0,Nu,1)=Umax_;
        Umin.block(i*Nu,0,Nu,1)=Umin_;
        deltaUmax.block(i*Nu,0,Nu,1)=deltaUmax_;
        deltaUmin.block(i*Nu,0,Nu,1)=deltaUmin_;
    }

    lb.block(0,0,Nc*Nu,1)=Umax-Ut;
    lu.block(0,0,Nc*Nu,1)=Umin-Ut;
    lb.block(Nc*Nu,0,Nc*Nu,1)=deltaUmax;
    lu.block(Nc*Nu,0,Nc*Nu,1)=deltaUmin;
    
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    int numOfVar=Nc*Nu+1;
    int numOfCons=2*Nc*Nu;
    solver.data()->setNumberOfVariables(numOfVar);
    solver.data()->setNumberOfConstraints(numOfCons);
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    hessian.resize(numOfVar, numOfVar);
    gradient.resize(numOfVar);
    linearMatrix.resize(numOfCons, numOfVar);
    lowerBound.resize(numOfCons);
    upperBound.resize(numOfCons);
    hessian=H.sparseView();
    gradient=g;
    linearMatrix=A_cons.sparseView();
    lowerBound=lu;
    upperBound=lb;
    
    // for (int i=0;i<H.rows();i++){
    //     for (int j=0;j<H.cols();j++){
    //         cout<<H(i,j);
    //     }
    //     cout<<endl;
    // }
    
    if (solver.data()->setHessianMatrix(hessian) && solver.data()->setGradient(gradient)&& 
        solver.data()->setLinearConstraintsMatrix(linearMatrix) &&
        solver.data()->setLowerBound(lowerBound) && 
        solver.data()->setUpperBound(upperBound)){
        solver.initSolver();
        //ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        solver.solve();
    } 
    
    Eigen::VectorXd QPSolution = solver.getSolution();
    double delta_v_title=QPSolution[0];
    double delta_throll_title=QPSolution[1];
    ROS_INFO("参考点x==%.2f  y==%.2f  yaw==%.2f  v==%.2f  throllr==%.2f",xr, yr, yawr,vr,throllr);
    ROS_INFO("delta_v_title=%.2f  delta_throll_title=%.2f",delta_v_title,delta_throll_title);
    uk_1[0]=kesi(3)+delta_v_title;
    uk_1[1]=kesi(4)+delta_throll_title;
    double v_out=uk_1[0]+vr;
    double throll_out=uk_1[1]+throllr;
    ROS_INFO("ego_position.yaw=%.2f",ego_position.yaw);
    ROS_INFO("ego_yaw-yawr=%.2f",ego_yaw-yawr);
    ROS_INFO("uk_1[0]=%.2f  uk_1[1]=%.2f",uk_1[0],uk_1[1]);
    ROS_INFO("kesi(3)=%.2f  kesi(4)=%.2f",kesi(3),kesi(4));
    if (indexx==reference_path.size()-1){
        v_out=0;
        throll_out=0;
    }
    ROS_INFO("V==%.2f   throll==%.2f",v_out,throll_out);
    
    vector<double> output{v_out,throll_out};
    
    update(output);
}













































































