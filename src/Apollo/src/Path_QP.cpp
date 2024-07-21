#include "Path_QP.h"

int FindNearIndex(const vector<double> &dp_path_s, double obs_s){
    int index=0;
    if (dp_path_s[0] >= obs_s){
        return index;
    }else if (dp_path_s[dp_path_s.size()-1] <= obs_s){
        index=59;
        return index;
    }else{
        while (dp_path_s[index] < obs_s){
            index+=1;
        }
        if (dp_path_s[index]-obs_s > obs_s-dp_path_s[index-1]){
            index-=1;
        }
        return index;
    }
}

PATH_QP_Solver::PATH_QP_Solver(double cost_l,double cost_dl,double cost_ddl,double cost_dddl,
                double cost_centre,double cost_end_l,double cost_end_dl,double cost_end_ddl,
                double d1,double d2,double w){
    w_cost_l=cost_l;
    w_cost_dl=cost_dl;
    w_cost_ddl=cost_ddl;
    w_cost_dddl=cost_dddl;
    w_cost_centre=cost_centre;
    w_cost_end_l=cost_end_l;
    w_cost_end_dl=cost_end_dl;
    w_cost_end_ddl=cost_end_ddl;
    host_d1=d1;
    host_d2=d2;
    host_w=w;

    l_min.resize(60,-8.0);
    l_max.resize(60,8.0);

    qp_path_s.resize(20,0.0);
    qp_path_l.resize(20,0.0);
    qp_path_dl.resize(20,0.0);
    qp_path_ddl.resize(20,0.0);

    qp_path_s_final.resize(501,0.0);
    qp_path_l_final.resize(501,0.0);
    qp_path_dl_final.resize(501,0.0);
    qp_path_ddl_final.resize(501,0.0);
}

void PATH_QP_Solver::Generate_Convex_Space(const vector<double> &dp_path_s,
                    const vector<double> &dp_path_l,const vector<double> &static_obs_s_set,
                    const vector<double> &static_obs_l_set){
    for (int i=0;i<60;i++){
        l_max[i]=8.0;
        l_min[i]=-8.0;
    }
    for (int i=0;i<dp_path_l.size();i++){
        std::cout<<"dp_path_l=="<<dp_path_l[i]<<std::endl;
    }
    std::cout<<"static_obs_s_set.size()="<<static_obs_s_set.size()<<std::endl;
    for (int i=0;i<static_obs_s_set.size();i++){
        if (std::isnan(static_obs_s_set[i])) break;
        double obs_s_min=static_obs_s_set[i] - static_obs_length/2.0;
        double obs_s_max=static_obs_s_set[i] + static_obs_length/2.0;
        std::cout<<"static_obs_s_set[i]="<<static_obs_s_set[i]<<std::endl;
        std::cout<<"static_obs_l_set[i]="<<static_obs_l_set[i]<<std::endl;
        int start_index=FindNearIndex(dp_path_s,obs_s_min);
        int end_index=FindNearIndex(dp_path_s,obs_s_max);
        std::cout<<"start_index="<<start_index<<"  end_index="<<end_index<<std::endl;
        double path_l=(dp_path_l[start_index] + dp_path_l[end_index]) / 2.0;
        if (start_index==0 && end_index==0) continue;
        if (path_l > static_obs_l_set[i]){
            for (int j=start_index;j<=end_index;j++){
                l_min[j]=max(l_min[j],static_obs_l_set[i]+static_obs_width/2.0);
            }
        }else{
            for (int j=start_index;j<=end_index;j++){
                l_max[j]=min(l_max[j],static_obs_l_set[i]-static_obs_width/2.0);
            }
        }
    }
    return;
    
}

void PATH_QP_Solver::Path_Planning_With_Quadratic_Programming(double plan_start_s,
                    double plan_start_l,double plan_start_dl,double plan_start_ddl){
    int n=20;
    for (int i=0;i<n;i++){
        qp_path_s[i]=0.0;
        qp_path_l[i]=0.0;
        qp_path_dl[i]=0.0;
        qp_path_ddl[i]=0.0;
    }
    double ds=3.0;
    Eigen::MatrixXd H_L=Eigen::MatrixXd::Zero(3*n,3*n);
    Eigen::MatrixXd H_DL=Eigen::MatrixXd::Zero(3*n,3*n);
    Eigen::MatrixXd H_DDL=Eigen::MatrixXd::Zero(3*n,3*n);
    Eigen::MatrixXd H_DDDL=Eigen::MatrixXd::Zero(n-1,3*n);
    Eigen::MatrixXd H_CENTRE=Eigen::MatrixXd::Zero(3*n,3*n);
    Eigen::MatrixXd H_L_END=Eigen::MatrixXd::Zero(3*n,3*n);
    Eigen::MatrixXd H_DL_END=Eigen::MatrixXd::Zero(3*n,3*n);
    Eigen::MatrixXd H_DDL_END=Eigen::MatrixXd::Zero(3*n,3*n);
    for (int i=0;i<n;i++){
        H_L(3*i , 3*i)=1;
        H_DL(3*i+1 , 3*i+1)=1;
        H_DDL(3*i+2 , 3*i+2)=1;
    }
    Eigen::RowVectorXd H_dddl_sub = Eigen::RowVectorXd::Zero(6);
    H_dddl_sub << 0, 0, 1.0/ds, 0, 0, -1.0/ds;
    for (int i=0;i<n-1;i++){
        int row=i;
        int col=3*i;
        H_DDDL.block(row,col,1,6)=H_dddl_sub;
    }
    H_CENTRE=H_L;
    H_L_END(3*n-3 , 3*n-3)=1;
    H_DL_END(3*n-2 , 3*n-2)=1;
    H_DDL_END(3*n-1 , 3*n-1)=1;
    Eigen::MatrixXd H=w_cost_l*(H_L.transpose()*H_L) + 
                      w_cost_dl*(H_DL.transpose()*H_DL) + 
                      w_cost_ddl*(H_DDL.transpose()*H_DDL) + 
                      w_cost_dddl*(H_DDDL.transpose()*H_DDDL) +
                      w_cost_centre*(H_CENTRE.transpose()*H_CENTRE) + 
                      w_cost_end_l*(H_L_END.transpose()*H_L_END) + 
                      w_cost_end_dl*(H_DL_END.transpose()*H_DL_END) + 
                      w_cost_end_ddl*(H_DDL_END.transpose()*H_DDL_END);
    H=2*H;

    Eigen::MatrixXd A_dl_maxormin=Eigen::MatrixXd::Zero(n-1,3*n);
    Eigen::VectorXd b_dl_min=Eigen::VectorXd::Zero(n-1);
    Eigen::VectorXd b_dl_max=Eigen::VectorXd::Zero(n-1);
    Eigen::MatrixXd A_ddl_maxormin=Eigen::MatrixXd::Zero(n-1,3*n);
    Eigen::VectorXd b_ddl_min=Eigen::VectorXd::Zero(n-1);
    Eigen::VectorXd b_ddl_max=Eigen::VectorXd::Zero(n-1);
    Eigen::RowVectorXd A_dl_sub=Eigen::RowVectorXd::Zero(6);
    A_dl_sub << 0, -1, 0, 0, 1, 0;
    Eigen::RowVectorXd A_ddl_sub=Eigen::RowVectorXd::Zero(6);
    A_ddl_sub << 0, 0, -1, 0, 0, 1;
    for (int i=0;i<n-1;i++){
        int row=i;
        int col=3*i;
        A_dl_maxormin.block(row,col,1,6)=A_dl_sub;
        A_ddl_maxormin.block(row,col,1,6)=A_ddl_sub;
        b_dl_min[row]=-delta_dl_max;
        b_dl_max[row]=delta_dl_max;
        b_ddl_min[row]=-delta_ddl_max;
        b_ddl_max[row]=delta_ddl_max;
    }
    //std::cout<<b_dl_min<<std::endl;
    Eigen::MatrixXd Aeq_sub=Eigen::MatrixXd::Zero(2,6);
    Aeq_sub << 1, ds, ds*ds/3.0, -1,  0,  ds*ds/6.0,
               0, 1,  ds/2.0,     0, -1,  ds/2.0;
    Eigen::MatrixXd Aeq=Eigen::MatrixXd::Zero(2*n-2,3*n);
    Eigen::VectorXd beq=Eigen::VectorXd::Zero(2*n-2);
    for (int i=0;i<n-1;i++){
        int row=2*i;
        int col=3*i;
        Aeq.block(row,col,2,6)=Aeq_sub;
    }
    
    Eigen::MatrixXd A=Eigen::MatrixXd::Zero(4*n,3*n);
    Eigen::VectorXd b_max=Eigen::VectorXd::Zero(4*n);
    Eigen::VectorXd b_min=Eigen::VectorXd::Zero(4*n);
    Eigen::MatrixXd A_sub=Eigen::MatrixXd::Zero(4,3);
    A_sub << 1, host_d1, 0,
             1, host_d1, 0,
             1, -host_d2, 0,
             1, -host_d2, 0;
    for (int i=1;i<n;i++){
        int row=4*i;
        int col=3*i;
        A.block(row,col,4,3)=A_sub;
    }
    
    int front_index=std::ceil(host_d1/ds);
    int back_index=std::ceil(host_d2/ds);

    for (int i=1;i<n;i++){
        int index1=std::min(3*i + front_index , 59);
        int index2=std::max(3*i - back_index , 0);
        b_max[4*i]    =l_max[index1] - host_w/2.0;
        b_max[4*i + 1]=l_max[index1] + host_w/2.0;
        b_max[4*i + 2]=l_max[index2] - host_w/2.0;
        b_max[4*i + 3]=l_max[index2] + host_w/2.0;

        b_min[4*i]    =l_min[index1] - host_w/2.0;
        b_min[4*i + 1]=l_min[index1] + host_w/2.0;
        b_min[4*i + 2]=l_min[index2] - host_w/2.0;
        b_min[4*i + 3]=l_min[index2] + host_w/2.0;
    }
    
    
    //起点约束
    Eigen::MatrixXd AA=Eigen::MatrixXd::Identity(3*n,3*n);
    Eigen::VectorXd lb=Eigen::VectorXd::Ones(3*n)*(-8);
    Eigen::VectorXd ub=Eigen::VectorXd::Ones(3*n)*(8);
    std::cout<<"plan_start_l=="<<plan_start_l;
    std::cout<<"  plan_start_dl=="<<plan_start_dl;
    std::cout<<"  plan_start_ddl=="<<plan_start_ddl<<std::endl;
    lb[0]=plan_start_l;
    lb[1]=plan_start_dl;
    lb[2]=plan_start_ddl;

    ub[0]=plan_start_l;
    ub[1]=plan_start_dl;
    ub[2]=plan_start_ddl;

    Eigen::MatrixXd A_cons;
    A_cons.resize(A_dl_maxormin.rows() + A_ddl_maxormin.rows() +
                    Aeq.rows() + A.rows() + AA.rows() , 3*n);
    A_cons.block(0, 0, A_dl_maxormin.rows(), A_dl_maxormin.cols()) = A_dl_maxormin;
    A_cons.block(A_dl_maxormin.rows(), 0, A_ddl_maxormin.rows(), A_ddl_maxormin.cols()) = A_ddl_maxormin;
    A_cons.block(A_dl_maxormin.rows() + A_ddl_maxormin.rows(), 0, Aeq.rows(), Aeq.cols()) = Aeq;
    A_cons.block(A_dl_maxormin.rows() + A_ddl_maxormin.rows() + Aeq.rows(), 0, A.rows(), A.cols()) = A;
    A_cons.block(A_dl_maxormin.rows() + A_ddl_maxormin.rows() + Aeq.rows() + A.rows(), 0, AA.rows(), AA.cols()) = AA;
    Eigen::VectorXd low_cons;
    low_cons.resize(b_dl_min.rows() + b_ddl_min.rows() + beq.rows() + b_min.rows() + lb.rows());
    low_cons.block(0, 0, b_dl_min.rows(), 1) = b_dl_min;
    low_cons.block(b_dl_min.rows(), 0, b_ddl_min.rows(), 1) = b_ddl_min;
    low_cons.block(b_dl_min.rows() + b_ddl_min.rows(), 0, beq.rows(), 1) = beq;
    low_cons.block(b_dl_min.rows() + b_ddl_min.rows() + beq.rows(), 0, b_min.rows(), 1) = b_min;
    low_cons.block(b_dl_min.rows() + b_ddl_min.rows() + beq.rows() + b_min.rows(), 0, lb.rows(), 1) = lb;

    Eigen::VectorXd upper_cons;
    upper_cons.resize(b_dl_max.rows() + b_ddl_max.rows() + beq.rows() + b_max.rows() + ub.rows());
    upper_cons.block(0, 0, b_dl_max.rows(), 1) = b_dl_max;
    upper_cons.block(b_dl_max.rows(), 0, b_ddl_max.rows(), 1) = b_ddl_max;
    upper_cons.block(b_dl_max.rows() + b_ddl_max.rows(), 0, beq.rows(), 1) = beq;
    upper_cons.block(b_dl_max.rows() + b_ddl_max.rows() + beq.rows(), 0, b_max.rows(), 1) = b_max;
    upper_cons.block(b_dl_max.rows() + b_ddl_max.rows() + beq.rows() + b_max.rows(), 0, ub.rows(), 1) = ub;
    
    Eigen::VectorXd f=Eigen::VectorXd::Zero(3*n);
    for (int i=0;i<n;i++){
        f[3*i]=-(l_max[3*i] + l_min[3*i]);
        if (abs(f[3*i]) > 0.3){
            f[3*i]=f[3*i]*w_cost_centre;
        }
    }
    double end_l_desire = 0.0;
    double end_dl_desire = 0.0;
    double end_ddl_desire = 0.0;
    f[3*n-3] = f[3*n-3] -2*end_l_desire*w_cost_end_l;
    f[3*n-2] = f[3*n-2] -2*end_dl_desire*w_cost_end_dl;
    f[3*n-1] = f[3*n-1] -2*end_ddl_desire*w_cost_end_ddl;
    //std::cout<<f<<std::endl;
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    int numOfVar=A_cons.cols();
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
    lowerBound=low_cons;
    upperBound=upper_cons;

    if (solver.data()->setHessianMatrix(hessian) && solver.data()->setGradient(gradient)&& 
        solver.data()->setLinearConstraintsMatrix(linearMatrix) &&
        solver.data()->setLowerBound(lowerBound) && 
        solver.data()->setUpperBound(upperBound)){
        solver.initSolver();
        solver.solve();
    } 
    Eigen::VectorXd QPSolution = solver.getSolution();
    for (int i=0;i<n;i++){
        qp_path_s[i]=plan_start_s + i*ds;
        //std::cout<<"sssss=="<<plan_start_s<<std::endl;
        qp_path_l[i]=QPSolution[3*i];
        std::cout<<"ss=="<<qp_path_s[i];
        std::cout<<"  ll="<<qp_path_l[i]<<std::endl;
        qp_path_dl[i]=QPSolution[3*i + 1];
        qp_path_ddl[i]=QPSolution[3*i + 2];
    }
    
    for (int i=0;i<l_max.size();i++){
        std::cout<<"l_max=="<<l_max[i];
        std::cout<<"l_min=="<<l_min[i]<<std::endl;
    }
}

void PATH_QP_Solver::Increase_QP_Node(){
    int n_init=20;
    int n=501;
    for (int i=0;i<n;i++){
        qp_path_s_final[i]=0.0;
        qp_path_l_final[i]=0.0;
        qp_path_dl_final[i]=0.0;
        qp_path_ddl_final[i]=0.0;
    }
    double ds=(qp_path_s[qp_path_s.size()-1] - qp_path_s[0])/(n-1);
    int index=0;
    for (int i=0;i<n;i++){
        double x=qp_path_s[0] + i*ds;
        qp_path_s_final[i]=x;
        while (x >= qp_path_s[index]){
            index+=1;
            if (index == n_init-1) break;
        }
        int pre=index-1;
        int cur=index;
        double delta_s=x-qp_path_s[pre];
        double l_pre=qp_path_l[pre];
        double dl_pre=qp_path_dl[pre];
        double ddl_pre=qp_path_ddl[pre];
        double ddl_cur=qp_path_ddl[cur];
        qp_path_l_final[i]=l_pre + dl_pre*delta_s + ddl_pre*pow(delta_s,2)/3.0 +
                            ddl_cur*pow(delta_s,2)/6.0;
        qp_path_dl_final[i]=dl_pre + 0.5*ddl_pre*delta_s + 0.5*ddl_cur*delta_s;
        qp_path_ddl_final[i]=ddl_pre + (ddl_cur-ddl_pre)*delta_s/(qp_path_s[cur]-qp_path_s[pre]);
        index-=1;
    }
    
}

std::tuple<vector<double>,vector<double>,
            vector<double>,vector<double>> PATH_QP_Solver::Get_QP_Result(){
    return std::make_tuple(qp_path_s_final,qp_path_l_final,qp_path_dl_final,qp_path_ddl_final);
}