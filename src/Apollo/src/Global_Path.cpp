#include "Global_Path.h"


void Ego_State_Update(Host_State& ego , double dt, double steer){
    double vx=ego.vx*cos(ego.heading_xy) - ego.vy*sin(ego.heading_xy);
    double vy=ego.vx*sin(ego.heading_xy) + ego.vy*cos(ego.heading_xy);
    double ax=ego.ax*cos(ego.heading_xy) - ego.ay*sin(ego.heading_xy);
    double ay=ego.ax*sin(ego.heading_xy) + ego.ay*cos(ego.heading_xy);
    ego.x += vx*dt + 0.5*ax*dt*dt;
    ego.y += vy*dt + 0.5*ay*dt*dt;
    ego.vx += ego.ax*dt;
    ego.vy += ego.ay*dt;
    double v=sqrt(pow(vx,2) + pow(vy,2));
    ego.heading_xy += v*tan(steer)*dt/L;

}

std::tuple<vector<double>, vector<double>> UnSmoothPath(const int &match_index,
                    vector<double> global_x, vector<double> global_y) {
    int start_index=0;
    int len=global_x.size();
    if (match_index-30 < 0) start_index=0;
    else if (match_index+150 >=len) start_index=len-181;
    else start_index=match_index-30;
    vector<double> reference_x,reference_y;
    for (int i=start_index;i<=start_index+180;i++){
        reference_x.push_back(global_x[i]);
        reference_y.push_back(global_y[i]);
    }
    return std::make_tuple(reference_x, reference_y);
}

int referenceline_provider_seek_match_index(const double &host_x, const double &host_y,
                const vector<double> &global_x, const vector<double> &global_y,
                const vector<double> &global_heading, const vector<double> &global_kappa){
    static bool init_run=true;
    static int pre_match_point_index_set=-1;
    static vector<double> pre_match_point;
    static vector<double> pre_match_point_direction;
    int len=global_x.size();
    if (init_run){
        int start_search_index=0;
        int increase_count=0;
        double min_distance=numeric_limits<double>::max();
        for (int j=start_search_index; j<len; j++){
            double distance=pow(host_x-global_x[j],2)+pow(host_y-global_y[j],2);
            if (distance < min_distance){
                min_distance=distance;
                pre_match_point_index_set=j;
                increase_count=0;
            }else{
                increase_count+=1;
            }
            if (increase_count>50) break;
        }
        init_run=false;
        pre_match_point={global_x[pre_match_point_index_set],
                         global_y[pre_match_point_index_set]};
        pre_match_point_direction={cos(global_heading[pre_match_point_index_set]),
                                    sin(global_heading[pre_match_point_index_set])};
    }else{
        int start_search_index=pre_match_point_index_set;
        int increase_count=0;
        int increase_count_limit=5;
        if (pre_match_point_index_set==-1){
            start_search_index=0;
            increase_count_limit=50;
        }
        double flag=(host_x-pre_match_point[0])*pre_match_point_direction[0]+
                        (host_y-pre_match_point[1])*pre_match_point_direction[1];
        double min_distance=numeric_limits<double>::max();
        if (flag > 0.001){
            for (int j=start_search_index;j<len;j++){
                double distance=pow(host_x-global_x[j],2) + pow(host_y-global_y[j],2);
                if (distance < min_distance){
                    min_distance=distance;
                    pre_match_point_index_set=j;
                    increase_count=0;
                }else{
                    increase_count+=1;
                }
                if (increase_count > increase_count_limit) break;
            }
        }else if (flag < -0.001){
            for (int j=start_search_index;j>=0;j--){
                double distance=pow(host_x-global_x[j],2) + pow(host_y-global_y[j],2);
                if (distance < min_distance){
                    min_distance=distance;
                    pre_match_point_index_set=j;
                    increase_count=0;
                }else{
                    increase_count+=1;
                }
                if (increase_count > increase_count_limit) break;
            }
        }else{
            pre_match_point_index_set=start_search_index;
        }
        pre_match_point={global_x[pre_match_point_index_set],
                         global_y[pre_match_point_index_set]};
        pre_match_point_direction={cos(global_heading[pre_match_point_index_set]),
                                    sin(global_heading[pre_match_point_index_set])};
    }
    int match_index=pre_match_point_index_set;
    return match_index;
}

std::tuple<vector<double>, vector<double>> Cal_heading_kappa(
                const vector<double> &reference_x,const vector<double> &reference_y){
    vector<double> dx, dy;
    for (int i=0; i<reference_x.size()-1; i++){
        double end_x=reference_x[i+1];
        double end_y=reference_y[i+1];
        double start_x=reference_x[i];
        double start_y=reference_y[i];
        double delta_x=end_x - start_x;
        double delta_y=end_y - start_y;
        dx.push_back(delta_x);
        dy.push_back(delta_y);
    }
    
    vector<double> dx_pre, dx_after, dx_final;
    vector<double> dy_pre, dy_after, dy_final;
    dx_pre.push_back(dx[0]);
    dy_pre.push_back(dy[0]);
    dx_pre.insert(dx_pre.end(),dx.begin(),dx.end());
    dy_pre.insert(dy_pre.end(),dy.begin(),dy.end());
    dx_after=dx;
    dy_after=dy;   
    dx_after.push_back(dx[dx.size()-1]);
    dy_after.push_back(dy[dy.size()-1]);
    
    for (int i=0;i<dx_after.size();i++){
        double xx=(dx_pre[i]+dx_after[i])/2.0;
        double yy=(dy_pre[i]+dy_after[i])/2.0;
        dx_final.push_back(xx);
        dy_final.push_back(yy);
    }
    
    vector<double> ds_final;
    vector<double> path_heading;
    for (int i=0;i<dx_final.size();i++){
        double ds=sqrt(pow(dx_final[i],2) + pow(dy_final[i],2));
        ds_final.push_back(ds);
        double head=atan2(dy_final[i],dx_final[i]);
        path_heading.push_back(head);
    }
    
    vector<double> dheading, dheading_pre, dheading_after, dheading_fianl;
    for (int i=0;i<path_heading.size()-1;i++){
        double start_head=path_heading[i];
        double end_head=path_heading[i+1];
        double delta_head=end_head - start_head;
        dheading.push_back(delta_head);
    }
    //ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    dheading_pre.push_back(dheading[0]);
    dheading_pre.insert(dheading_pre.end(),dheading.begin(),dheading.end());
    dheading_after=dheading;
    dheading_after.push_back(dheading[dheading.size()-1]);

    for (int i=0;i<dheading_after.size();i++){
        double hh=(dheading_pre[i] + dheading_after[i])/2.0;
        dheading_fianl.push_back(hh);
    }
    
    vector<double> path_kappa;
    for (int i=0; i<dheading_after.size(); i++){
        double kk=sin(dheading_fianl[i])/ds_final[i];
        path_kappa.push_back(kk);
    }
    
    return std::make_tuple(path_heading,path_kappa);

}

std::tuple<vector<double>,vector<double>> generate_globalpath(){
    vector<double> global_x,global_y;
    double i=0.0;
    while (i<500000){
        double xx=i/10.0;
        double yy=0.0;
        global_x.push_back(xx);
        global_y.push_back(yy);
        i+=5.0;
    }
    // double j=0.0;
    // while (j<500){
    //     double xx=50.0;
    //     double yy=j/10.0;
    //     global_x.push_back(xx);
    //     global_y.push_back(yy);
    //     j+=5.0;
    // }
    // double k=0.0;
    // while (k<500){
    //     double xx=50.0+k/10.0;
    //     double yy=50.0;
    //     global_x.push_back(xx);
    //     global_y.push_back(yy);
    //     k+=5.0;
    // }
    // double u=0.0;
    // while (u<500){
    //     double xx=100.0;
    //     double yy=50.0-u/10.0;
    //     global_x.push_back(xx);
    //     global_y.push_back(yy);
    //     u+=5.0;
    // }
    // double ii=0.0;
    // while (ii<500){
    //     double xx=100.0+ii/10.0;
    //     double yy=0.0;
    //     global_x.push_back(xx);
    //     global_y.push_back(yy);
    //     ii+=5.0;
    // }
    // double circle_center[2]={150.0, 50.0};
    // double init_angle=-M_PI/2;
    // while (init_angle < M_PI/2){
    //     double dx=circle_center[0] + 50.0*cos(init_angle);
    //     double dy=circle_center[1] + 50.0*sin(init_angle);
    //     global_x.push_back(dx);
    //     global_y.push_back(dy);
    //     init_angle+=M_PI/1000;
    // }
    // int kk=0;
    // while (kk<1500){
    //     double xx=150.0-kk/10.0;
    //     double yy=100.0;
    //     global_x.push_back(xx);
    //     global_y.push_back(yy);
    //     kk+=5.0;
    // }
    return std::make_tuple(global_x,global_y);
}

vector<GlobalNode> Get_SmoothReference(const vector<double> &reference_x,
                                        const vector<double> &reference_y){
    auto smooth_headkappa=Cal_heading_kappa(reference_x,reference_y);
    vector<double> reference_heading=std::get<0>(smooth_headkappa);
    vector<double> reference_kappa=std::get<1>(smooth_headkappa);
    vector<GlobalNode> global;
    for (int i=0;i<reference_x.size();i++){
        GlobalNode Node;
        Node.x=reference_x[i];
        Node.y=reference_y[i];
        Node.heading=reference_heading[i];
        Node.kappa=reference_kappa[i];
        global.push_back(Node);
    }
    return global;
}

std::tuple<vector<double>,vector<double>> Smooth_Globalpath(vector<double> reference_x,
                                vector<double> reference_y){
    int n=reference_x.size();
    
    //std::cout<<"n1=="<<reference_x.size()<<"  n2=="<<reference_y.size()<<std::endl;
    Eigen::MatrixXd A1=Eigen::MatrixXd::Zero(2*n,2*n-4);
    Eigen::MatrixXd A2=Eigen::MatrixXd::Zero(2*n,2*n-2);
    Eigen::MatrixXd A3=Eigen::MatrixXd::Identity(2*n,2*n);
    Eigen::MatrixXd a1=Eigen::MatrixXd::Zero(6, 2);
    a1 << 1, 0,
          0, 1,
         -2, 0,
          0, -2,
          1, 0,
          0, 1;
    Eigen::MatrixXd a2=Eigen::MatrixXd::Zero(4,2);
    a2 << 1, 0,
          0, 1,
         -1, 0,
          0,-1;
    for (int i=0;i<n-2;i++){
        A1.block(2*i,2*i,6,2)=a1;
    }
    
    for (int j=0;j<n-1;j++){
        A2.block(2*j,2*j,4,2)=a2;
    }
    //std::cout<<A1;
    
    Eigen::MatrixXd HH=2*SMOOTH_COST*A1*A1.transpose() + 
                        2*LENGTH_COST*A2*A2.transpose() + 2*REF_COST*A3*A3.transpose();
    
    
    Eigen::VectorXd g=Eigen::VectorXd::Zero(2*n);
    Eigen::VectorXd Xref=Eigen::VectorXd::Zero(2*n);
    
    for (int k=0;k<n;k++){
        double xr=reference_x[k];
        double yr=reference_y[k];
        //std::cout<<"xr=="<<xr<<"    yr=="<<yr<<std::endl;
        g[2*k]=-2.0*REF_COST*xr;
        g[2*k+1]=-2.0*REF_COST*yr;
        Xref[2*k]=xr;
        Xref[2*k+1]=yr;
    }
    
    //std::cout<<g<<std::endl;
    Eigen::MatrixXd A_cons=Eigen::MatrixXd::Identity(2*n,2*n);
    
    Eigen::VectorXd lb=Eigen::VectorXd::Ones(2*n)*BUFF_LB + Xref;
    Eigen::VectorXd ub=Eigen::VectorXd::Ones(2*n)*BUFF_UB + Xref;
    //std::cout<<lu<<std::endl;
    
    OsqpEigen::Solver Ssolver;
    Ssolver.settings()->setWarmStart(true);
    int numOfVar=2*n;
    int numOfCons=2*n;
    Ssolver.data()->setNumberOfVariables(numOfVar);
    Ssolver.data()->setNumberOfConstraints(numOfCons);
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
    hessian=HH.sparseView();
    gradient=g;
    linearMatrix=A_cons.sparseView();
    lowerBound=lb;
    upperBound=ub;
    
    if (Ssolver.data()->setHessianMatrix(hessian) && 
        Ssolver.data()->setGradient(gradient)&& 
        Ssolver.data()->setLinearConstraintsMatrix(linearMatrix) &&
        Ssolver.data()->setLowerBound(lowerBound) && 
        Ssolver.data()->setUpperBound(upperBound)){

        Ssolver.initSolver();
        Ssolver.solve();
    } 
    Eigen::VectorXd QPSolution = Ssolver.getSolution();
    vector<double> smooth_x,smooth_y;
    for (int ii=0;ii<n;ii++){
        smooth_x.push_back(QPSolution[2*ii]);
        smooth_y.push_back(QPSolution[2*ii+1]);
    }
    
    std::cout<<"n1=="<<smooth_x.size()<<"  n2=="<<smooth_y.size()<<std::endl;
    return std::make_tuple(smooth_x,smooth_y);
}