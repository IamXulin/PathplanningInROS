#include "SpeedPlanning.h"

double interpolate(double x_interp, const vector<double> &x_values,
                                const vector<double> &y_values){

        if (x_values.size() != y_values.size()) {
            throw std::invalid_argument("Input vectors must have the same size");
        }
        if (x_interp <= x_values[0]) return y_values[0];
        if (x_interp >= x_values[x_values.size()-1]) return y_values[x_values.size()-1];
        auto it = std::upper_bound(x_values.begin(), x_values.end(), x_interp);


        if (it == x_values.begin() || it == x_values.end()) {
            throw std::out_of_range("Interpolation point is out of range");
        }

        size_t index = std::distance(x_values.begin(), it);
        double x1 = x_values[index - 1];
        double x2 = x_values[index];
        double y1 = y_values[index - 1];
        double y2 = y_values[index];

        return y1 + (x_interp - x1) * (y2 - y1) / (x2 - x1);
}

std::tuple<double,vector<double>> Calc_Path_Length_And_Map_Between_Point_And_S(
            const vector<double> &trajectory_x, const vector<double> &trajectory_y){
    int n=trajectory_x.size();
    vector<double> path_s(n,0.0);
    double sum=0.0;
    int index;
    for (int i=1;i<n;i++){
        if (std::isnan(trajectory_x[i])){
            index=i;
            break;
        }
        sum+=sqrt(pow(trajectory_x[i]-trajectory_x[i-1],2) + pow(trajectory_y[i]-trajectory_y[i-1],2));
        path_s[i]=sum;
    }

    double path_s_end;
    if (index==n-1){
        path_s_end=path_s[n-1];
    }else{
        path_s_end=path_s[index-1];
    }
    return std::make_tuple(path_s_end,path_s);

}

std::tuple<double,double> Calc_Speed_Planning_Start_Condition(const double &plan_start_vx,
                    const double &plan_start_vy, const double &plan_start_ax,
                    const double &plan_start_ay, const double &plan_start_heading){
    double tor[2]={cos(plan_start_heading) , sin(plan_start_heading)};
    double plan_start_s_dot=tor[0]*plan_start_vx + tor[1]*plan_start_vy;
    double plan_start_s_dot2=tor[0]*plan_start_ax + tor[1]*plan_start_ay;
    return std::make_tuple(plan_start_s_dot , plan_start_s_dot2);
}

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>> Sample_Speed_Qp_Planning(const double &plan_start_s_dot,
            const double &plan_start_s_dot2, const double &s_end, const double &recomment_T){
    int n=51;
    double dt=recomment_T/(n-1);
    vector<double> s_init(n,0.0);
    vector<double> s_dot_init(n,0.0);
    vector<double> s_dot2_init(n,0.0);
    vector<double> relative_time_init(n,0.0);
    std::cout<<"plan_start_s_dot=="<<plan_start_s_dot<<
        "  plan_start_s_dot2=="<<plan_start_s_dot2<<std::endl;
    Eigen::MatrixXd Aeq=Eigen::MatrixXd::Zero(2*n-2 , 3*n);
    Eigen::VectorXd beq=Eigen::VectorXd::Zero(2*n-2);
    Eigen::MatrixXd Aeq_sub;
    Aeq_sub.resize(2,6);
    Aeq_sub << 1, dt, dt*dt/3.0, -1, 0, dt*dt/6.0,
               0, 1, dt/2.0, 0, -1, dt/2.0;
    for (int i=0;i<n-1;i++){
        int row=2*i;
        int col=3*i;
        Aeq.block(row,col,2,6)=Aeq_sub;
    }
    Eigen::MatrixXd A=Eigen::MatrixXd::Identity(3*n,3*n);
    Eigen::VectorXd lb=Eigen::VectorXd::Zero(3*n);
    Eigen::VectorXd ub=Eigen::VectorXd::Zero(3*n);
    for (int i=0;i<n;i++){
        lb[3*i]=std::numeric_limits<double>::min();
        lb[3*i+1]=0.50;
        lb[3*i+2]=-4.0;
        ub[3*i]=std::numeric_limits<double>::max();
        ub[3*i+1]=30.0;
        ub[3*i+2]=6.0;
    }
    lb[0]=0.0;
    lb[1]=plan_start_s_dot;
    lb[2]=plan_start_s_dot2;
    ub[0]=0.0;
    ub[1]=plan_start_s_dot;
    ub[2]=plan_start_s_dot2;
    lb[3*n-3]=s_end;
    ub[3*n-3]=s_end;
    // std::cout<<"plan_start_s_dot=="<<plan_start_s_dot<<std::endl;
    // std::cout<<"plan_start_s_dot2=="<<plan_start_s_dot2<<std::endl;
    // std::cout<<"s_end"<<s_end<<std::endl;
    Eigen::MatrixXd A3=Eigen::MatrixXd::Identity(3*n,3*n);
    Eigen::MatrixXd A4=Eigen::MatrixXd::Zero(n-1,3*n);
    Eigen::RowVectorXd A4_sub=Eigen::RowVectorXd::Zero(6);
    A4_sub << 0, 0, 1, 0, 0, -1;
    for (int i=0;i<n-1;i++){
        int row=i;
        int col=3*i;
        A4.block(row,col,1,6)=A4_sub;
    }

    Eigen::MatrixXd H=A3 + 10*(A4.transpose()*A4);
    H=2*H;
    Eigen::VectorXd f=Eigen::VectorXd::Zero(3*n);
    Eigen::MatrixXd A_cons;
    Eigen::VectorXd lower_cons;
    Eigen::VectorXd upper_cons;
    A_cons.resize(Aeq.rows() + A.rows() , 3*n);
    lower_cons.resize(beq.rows() + lb.rows());
    upper_cons.resize(beq.rows() + ub.rows());
    A_cons.block(0 , 0 , Aeq.rows() , Aeq.cols())=Aeq;
    A_cons.block(Aeq.rows() , 0 , A.rows() , A.cols())=A;
    lower_cons.block(0 , 0 , beq.rows() , 1)=beq;
    lower_cons.block(beq.rows(), 0 , lb.rows() , 1)=lb;
    upper_cons.block(0 , 0 , beq.rows() , 1)=beq;
    upper_cons.block(beq.rows() , 0 , ub.rows() , 1)=ub;

    //std::cout<<"A_cons="<<A_cons.rows()<<std::endl;
    //std::cout<<"A="<<A<<std::endl;
    //std::cout<<"upper_cons="<<upper_cons.rows()<<std::endl;
    //std::cout<<Aeq<<std::endl;
    
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    int numOfVar=3*n;
    int numOfCons=A_cons.rows();
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
    gradient=f;
    linearMatrix=A_cons.sparseView();
    lowerBound=lower_cons;
    upperBound=upper_cons;
    
    if (solver.data()->setHessianMatrix(hessian) && 
        solver.data()->setGradient(gradient)&& 
        solver.data()->setLinearConstraintsMatrix(linearMatrix) &&
        solver.data()->setLowerBound(lowerBound) && 
        solver.data()->setUpperBound(upperBound)){

        solver.initSolver();
        solver.solve();
    } 
    
    Eigen::VectorXd QPSolution = solver.getSolution();
    
    for (int i=0;i<n;i++){
        s_init[i]=QPSolution[3*i];
        s_dot_init[i]=QPSolution[3*i+1];
        std::cout<<"s_dot=="<<s_dot_init[i]<<std::endl;
        s_dot2_init[i]=QPSolution[3*i+2];
        relative_time_init[i]=i*dt;
        //std::cout<<"tt=="<<relative_time_init[i]<<"   s_dot=="<<s_dot_init[i]<<std::endl;
    }
    return std::make_tuple(s_init, s_dot_init, s_dot2_init, relative_time_init);
}

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>> Increase_St_Point_Count(
    const vector<double> &s_init, const vector<double> &s_dot_init, 
    const vector<double> &s_dot2_init, const vector<double> &relative_time_init){
    double T=relative_time_init[relative_time_init.size()-1];
    int n=401;
    double dt=T/(n-1);
    vector<double> s(n,0.0);
    vector<double> s_dot(n,0.0);
    vector<double> s_dot2(n,0.0);
    vector<double> relative_time(n,0.0);
    
    //ROS_INFO("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY");
    //std::cout<<"relative_time_init=="<<relative_time_init.size()<<std::endl;
    for (int i=0;i<n;i++){
        double current_t=i*dt;
        int index;
        for (int j=0;j<relative_time_init.size()-1;j++){
            if (relative_time_init[j] <= current_t && relative_time_init[j+1] > current_t){
                index=j;
                break;
            }
        }
        //ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        double x=current_t - relative_time[index];
        s[i]=s_init[index] + s_dot_init[index]*x + s_dot2_init[index]*x*x/3.0 +
                             s_dot2_init[index+1]*x*x/6.0;
        s_dot[i]=s_dot_init[index] + 0.5*s_dot2_init[index]*x + 0.5*s_dot2_init[index+1]*x;
        s_dot2[i]=s_dot2_init[index] + (s_dot2_init[index+1] - s_dot2_init[index])*x /
                (relative_time_init[index+1] - relative_time_init[index]);
        relative_time[i]=current_t;
    }
    
    return std::make_tuple(s, s_dot, s_dot2, relative_time);
}

Plan_Trajectory Path_And_Speed_Merge(const vector<double> &s, const vector<double> &s_dot, 
                    const vector<double> &s_dot2, const vector<double> &relative_time, 
                    const double &current_time, const vector<double> &path_s, 
                    const vector<double> &trajectory_x_init, 
                    const vector<double> &trajectory_y_init,
                    const vector<double> &trajectory_heading_init, 
                    const vector<double> &trajectory_kappa_init){
    //path有600个点，speed有401个点，需要插值
    int n=401;
    vector<double> trajectory_x(n,0.0);
    vector<double> trajectory_y(n,0.0);
    vector<double> trajectory_heading(n,0.0);
    vector<double> trajectory_kappa(n,0.0);
    vector<double> trajectory_speed(n,0.0);
    vector<double> trajectory_accel(n,0.0);
    vector<double> trajectory_time(n,0.0);
    
    int index=0;
    while (!std::isnan(trajectory_x_init[index])){
        index++;
    }
    
    index-=1;
    size_t startIndex = 0;
    size_t endIndex = index;
    std::vector<double> extractedPath_s(path_s.begin() + startIndex, path_s.begin() + endIndex + 1);
    std::vector<double> extractedTrajectory_x(trajectory_x_init.begin() + startIndex,
                                    trajectory_x_init.begin()+endIndex + 1);
    // for (int i=0;i<s.size();i++){
    //     std::cout<<s[i]<<std::endl;
    // }
    std::vector<double> extractedTrajectory_y(trajectory_y_init.begin() + startIndex,
                                    trajectory_y_init.begin()+endIndex + 1);
    std::vector<double> extractedTrajectory_heading(trajectory_heading_init.begin() + startIndex,
                                    trajectory_heading_init.begin()+endIndex + 1);
    std::vector<double> extractedTrajectory_kappa(trajectory_kappa_init.begin() + startIndex,
                                    trajectory_kappa_init.begin()+endIndex + 1);
    // for (int i=0;i<extractedTrajectory_x.size();i++){
    //     std::cout<<"sssssssssssssss==="<<extractedPath_s[i];
    //     std::cout<<"      xxxxxxxxxxxxxxx==="<<extractedTrajectory_x[i]<<std::endl;
    // }
    for (int i=0;i<n-1;i++){
        
        trajectory_x[i]=interpolate(s[i], extractedPath_s, extractedTrajectory_x);
        trajectory_y[i]=interpolate(s[i], extractedPath_s, extractedTrajectory_y);
        trajectory_heading[i]=interpolate(s[i], extractedPath_s, extractedTrajectory_heading);
        trajectory_kappa[i]=interpolate(s[i], extractedPath_s, extractedTrajectory_kappa);
        trajectory_time[i]=relative_time[i]+current_time;
        trajectory_speed[i]=s_dot[i];
        trajectory_accel[i]=s_dot2[i];
    }

    //端点单独处理
    trajectory_x[n-1]=trajectory_x_init[trajectory_x_init.size()-1];
    trajectory_y[n-1]=trajectory_y_init[trajectory_y_init.size()-1];
    trajectory_heading[n-1]=trajectory_heading_init[trajectory_heading_init.size()-1];
    trajectory_kappa[n-1]=trajectory_kappa_init[trajectory_kappa_init.size()-1];
    trajectory_time[n-1]=relative_time[relative_time.size()-1]+current_time;
    trajectory_speed[n-1]=s_dot[s_dot.size()-1];
    trajectory_accel[n-1]=s_dot2[s_dot2.size()-1];

    Plan_Trajectory Trajectory;
    Trajectory.trajectory_x=trajectory_x;
    Trajectory.trajectory_y=trajectory_y;
    Trajectory.trajectory_heading=trajectory_heading;
    Trajectory.trajectory_kappa=trajectory_kappa;
    Trajectory.trajectory_velocity=trajectory_speed;
    Trajectory.trajectory_accel=trajectory_accel;
    Trajectory.trajectory_time=trajectory_time;
    
    return Trajectory;
}

Plan_Trajectory Stitch_Trajectory(const Plan_Trajectory &trajectory, 
                                    const Plan_Trajectory &stitch){
    Plan_Trajectory Trajectory;
    Trajectory.trajectory_x=stitch.trajectory_x;
    Trajectory.trajectory_x.insert(Trajectory.trajectory_x.end(),
                        trajectory.trajectory_x.begin(),trajectory.trajectory_x.end());

    Trajectory.trajectory_y=stitch.trajectory_y;
    Trajectory.trajectory_y.insert(Trajectory.trajectory_y.end(),
                        trajectory.trajectory_y.begin(),trajectory.trajectory_y.end());

    Trajectory.trajectory_heading=stitch.trajectory_heading;
    Trajectory.trajectory_heading.insert(Trajectory.trajectory_heading.end(),
            trajectory.trajectory_heading.begin(),trajectory.trajectory_heading.end());

    Trajectory.trajectory_kappa=stitch.trajectory_kappa;
    Trajectory.trajectory_kappa.insert(Trajectory.trajectory_kappa.end(),
                trajectory.trajectory_kappa.begin(),trajectory.trajectory_kappa.end());

    Trajectory.trajectory_velocity=stitch.trajectory_velocity;
    Trajectory.trajectory_velocity.insert(Trajectory.trajectory_velocity.end(),
            trajectory.trajectory_velocity.begin(),trajectory.trajectory_velocity.end());

    Trajectory.trajectory_accel=stitch.trajectory_accel;
    Trajectory.trajectory_accel.insert(Trajectory.trajectory_accel.end(),
                trajectory.trajectory_accel.begin(),trajectory.trajectory_accel.end());

    Trajectory.trajectory_time=stitch.trajectory_time;
    Trajectory.trajectory_time.insert(Trajectory.trajectory_time.end(),
                trajectory.trajectory_time.begin(),trajectory.trajectory_time.end());
    

    return Trajectory;
}