#include "Lattice.h"


double CalcObsCost(double w_cost_collision, Eigen::VectorXd &square_d){
    double min_d=std::numeric_limits<double>::max();
    for (int i=0;i<square_d.rows();i++){
        min_d=std::min(min_d , square_d[i]);
    }
    return w_cost_collision*pow(10 , -min_d/3.0);
}


Eigen::VectorXd CalcQuinticCoeffient(double start_l,double start_dl,double start_ddl,
            double end_l,double end_dl,double end_ddl,double start_s,double end_s){
    double start_s2=start_s * start_s;
    double start_s3=start_s2 * start_s;
    double start_s4=start_s3 * start_s;
    double start_s5=start_s4 * start_s;
    double end_s2=end_s * end_s;
    double end_s3=end_s2 * end_s;
    double end_s4=end_s3 * end_s;
    double end_s5=end_s4 * end_s;
    Eigen::MatrixXd A=Eigen::MatrixXd::Zero(6,6);
    A << 1 , start_s ,  start_s2  ,  start_s3  , start_s4    , start_s5,
         0 ,   1     ,  2*start_s , 3*start_s2 , 4*start_s3  , 5*start_s4,
         0 ,   0     ,     2      , 6*start_s  , 12*start_s2 , 20*start_s3,
         1 ,  end_s  ,   end_s2   ,  end_s3    ,   end_s4    , end_s5,
         0 ,   1     ,   2*end_s  , 3*end_s2   , 4*end_s3    , 5*end_s4,
         0 ,   0     ,     2      , 6*end_s    , 12*end_s2   , 20*end_s3;
    Eigen::VectorXd B=Eigen::VectorXd::Zero(6);
    B << start_l,
         start_dl,
         start_ddl,
         end_l,
         end_dl,
         end_ddl;
    Eigen::VectorXd coeff = A.inverse() * B;
    return coeff;
}

double CalcStartCost(const double &begin_s,const double &begin_l,
        const double &begin_dl,const double &begin_ddl,int cur_node_row,
        double sample_s,double sample_l,double w_cost_collision,double w_cost_smooth_dl,
        double w_cost_smooth_ddl,double w_cost_smooth_dddl,double w_cost_ref,
        const vector<double> &obs_s_set,const vector<double> &obs_l_set,int row){
    double start_l=begin_l;
    double start_dl=begin_dl;
    double start_ddl=begin_ddl;

    double end_l=((row - 1)/2 - cur_node_row)*sample_l;
    double end_dl=0.0;
    double end_ddl=0.0;

    double start_s=begin_s;
    double end_s=begin_s + sample_s;

    Eigen::VectorXd coeff=CalcQuinticCoeffient(start_l,start_dl,start_ddl,
                                                end_l,end_dl,end_ddl,start_s,end_s);
    double a0=coeff(0);
    double a1=coeff(1);
    double a2=coeff(2);
    double a3=coeff(3);
    double a4=coeff(4);
    double a5=coeff(5);

    Eigen::VectorXd ds=Eigen::VectorXd::Zero(10);
    Eigen::VectorXd l=Eigen::VectorXd::Zero(10);
    Eigen::VectorXd dl=Eigen::VectorXd::Zero(10);
    Eigen::VectorXd ddl=Eigen::VectorXd::Zero(10);
    Eigen::VectorXd dddl=Eigen::VectorXd::Zero(10);
    for (int i=0;i<10;i++){
        ds(i)=start_s + i*sample_s/10.0;
    }
    //Eigen::VectorXd result = vec.array().pow(3);
    l=a0 + a1*ds.array() + a2*ds.array().pow(2) + a3*ds.array().pow(3) +
      a4*ds.array().pow(4) + a5*ds.array().pow(5);
    dl=a1 + 2*a2*ds.array() + 3*a3*ds.array().pow(2) + 4*a4*ds.array().pow(3) +
       5*a5*ds.array().pow(4);
    ddl=2*a2 + 6*a3*ds.array() + 12*a4*ds.array().pow(2) +
        20*a5*ds.array().pow(3);
    dddl=6*a3 + 24*a4*ds.array() + 60*a5*ds.array().pow(2);
    double cost_smooth=w_cost_smooth_dl*dl.transpose().dot(dl) +
                       w_cost_smooth_ddl*ddl.transpose().dot(ddl) +
                       w_cost_smooth_dddl*dddl.transpose().dot(dddl);
    for (int i=0;i<dl.cols();i++){
        if (abs(ddl(i)) >0.5 || abs(atan(dl(i))) >0.4*M_PI){
            cost_smooth += 1e7;
        }
    } 
    double cost_ref=w_cost_ref*l.transpose().dot(l);
    double cost_collision=0.0;
    for (int i=0;i<obs_s_set.size();i++){
        if (isnan(obs_s_set[i])) break;

        Eigen::VectorXd dlon=Eigen::VectorXd::Ones(10)*obs_s_set[i] - ds;
        Eigen::VectorXd dlat=Eigen::VectorXd::Ones(10)*obs_l_set[i] - l;

        //认为障碍物是一个质点
        Eigen::VectorXd square_d=dlon.array().pow(2) + dlat.array().pow(2);
        double cost_collision_once=CalcObsCost(w_cost_collision,square_d);
        cost_collision +=cost_collision_once;
        double v1[2]={ds.tail(1)(0)-ds(0) , l.tail(1)(0)-l(0)};
        double v2[2]={obs_s_set[i]-ds(0) , obs_l_set[i]-l(0)};
        double flag=v1[0]*v2[1] - v1[1]*v2[0];
        if (flag <0 && cost_collision >0){
            cost_collision -=1000;
        }
    }
    double cost=cost_collision + cost_smooth + cost_ref;
    return cost;

}

double CalcNeighbourCost(double pre_node_s,double pre_node_l,double cur_node_s,
                        double cur_node_l,double w_cost_collision,double w_cost_smooth_dl,
                double w_cost_smooth_ddl,double w_cost_smooth_dddl,double w_cost_ref,
            const vector<double> &obs_s_set,const vector<double> &obs_l_set){
    double start_l=pre_node_l;
    double start_dl=0.0;
    double start_ddl=0.0;
    double end_l=cur_node_l;
    double end_dl=0.0;
    double end_ddl=0.0;
    double start_s=pre_node_s;
    double end_s=cur_node_s;
    Eigen::VectorXd coeff=CalcQuinticCoeffient(start_l, start_dl, start_ddl, end_l, 
                                                end_dl, end_ddl, start_s, end_s);
    double a0=coeff(0);
    double a1=coeff(1);
    double a2=coeff(2);
    double a3=coeff(3);
    double a4=coeff(4);
    double a5=coeff(5);

    Eigen::VectorXd ds=Eigen::VectorXd::Zero(10);
    Eigen::VectorXd l=Eigen::VectorXd::Zero(10);
    Eigen::VectorXd dl=Eigen::VectorXd::Zero(10);
    Eigen::VectorXd ddl=Eigen::VectorXd::Zero(10);
    Eigen::VectorXd dddl=Eigen::VectorXd::Zero(10);
    for (int i=0;i<10;i++){
        ds(i)=start_s + i*(end_s - start_s)/10.0;
    }
    //Eigen::VectorXd result = vec.array().pow(3);
    l=a0 + a1*ds.array() + a2*ds.array().pow(2) + a3*ds.array().pow(3) +
      a4*ds.array().pow(4) + a5*ds.array().pow(5);
    dl=a1 + 2*a2*ds.array() + 3*a3*ds.array().pow(2) + 4*a4*ds.array().pow(3) +
       5*a5*ds.array().pow(4);
    ddl=2*a2 + 6*a3*ds.array() + 12*a4*ds.array().pow(2) +
        20*a5*ds.array().pow(3);
    dddl=6*a3 + 24*a4*ds.array() + 60*a5*ds.array().pow(2);

    double cost_smooth = w_cost_smooth_dl * dl.transpose().dot(dl) +
                     w_cost_smooth_ddl * ddl.transpose().dot(ddl) +
                     w_cost_smooth_dddl * dddl.transpose().dot(dddl);
    for (int i=0;i<dl.cols();i++){
        if (abs(ddl(i)) >0.5 || abs(atan(dl(i))) >0.4*M_PI){
            cost_smooth += 1e7;
        }
    } 
    double cost_ref=w_cost_ref*l.transpose().dot(l);
    double cost_collision=0.0;
    for (int i=0;i<obs_s_set.size();i++){
        if (isnan(obs_s_set[i])) break;

        Eigen::VectorXd dlon=Eigen::VectorXd::Ones(10)*obs_s_set[i] - ds;
        Eigen::VectorXd dlat=Eigen::VectorXd::Ones(10)*obs_l_set[i] - l;

        //认为障碍物是一个质点
        Eigen::VectorXd square_d=dlon.array().pow(2) + dlat.array().pow(2);
        double cost_collision_once=CalcObsCost(w_cost_collision,square_d);
        cost_collision +=cost_collision_once;
        double v1[2]={ds.tail(1)(0)-ds(0) , l.tail(1)(0)-l(0)};
        double v2[2]={obs_s_set[i]-ds(0) , obs_l_set[i]-l(0)};
        double flag=v1[0]*v2[1] - v1[1]*v2[0];
        if (flag <0 && cost_collision >0){
            cost_collision -=1000;
        }
    }
    double cost=cost_collision + cost_smooth + cost_ref;
    return cost;
}

Lattice::Lattice(double cost_collision,double cost_dl,double cost_ddl,double cost_dddl,
                    double cost_ref,int row,int col,int sample_s,int sample_l){
    dp_cost_collision=cost_collision;
    dp_cost_smooth_dl=cost_dl;
    dp_cost_smooth_ddl=cost_ddl;
    dp_cost_smooth_dddl=cost_dddl;
    dp_cost_ref=cost_ref;
    dp_row=row;
    dp_col=col;
    dp_sample_s=sample_s;
    dp_sample_l=sample_l;

    dp_path_s_final.resize(60,std::nan(""));
    dp_path_l_final.resize(60,std::nan(""));
    dp_path_dl_final.resize(60,std::nan(""));
    dp_path_ddl_final.resize(60,std::nan(""));
}
std::tuple<vector<double>,vector<double>,vector<double>,vector<double>>
                                                 Lattice::Get_SLDLDDL(){
    return std::make_tuple(dp_path_s_final,dp_path_l_final,
                           dp_path_dl_final,dp_path_ddl_final);
}
std::tuple<vector<double>,vector<double>> Lattice::Dynamic_Planning(
        const vector<double> &static_obs_s_set,const vector<double> &static_obs_l_set,
        const double &plan_start_s,const double &plan_start_l,const double &plan_start_dl,
        const double &plan_start_ddl){
    double node_cost[dp_row][dp_col];
    int pre_node_index[dp_row][dp_col];
    for (int i=0;i<dp_row;i++){
        for (int j=0;j<dp_col;j++){
            node_cost[i][j]=numeric_limits<double>::max();
            pre_node_index[i][j]=0;
        }
    }
    for (int i=0;i<dp_row;i++){
        double end_l=((dp_row - 1)/2 - i)*dp_sample_l;
        if (std::abs(end_l) >= 8){
                ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
            }
        node_cost[i][0]=CalcStartCost(plan_start_s,plan_start_l,plan_start_dl,
                        plan_start_ddl,i,dp_sample_s,dp_sample_l,dp_cost_collision,
                        dp_cost_smooth_dl,dp_cost_smooth_ddl,dp_cost_smooth_dddl,
                        dp_cost_ref,static_obs_s_set,static_obs_l_set,dp_row);
    }
    //动态规划
    for (int j=1;j<dp_col;j++){
        for (int i=0;i<dp_row;i++){
            double cur_node_s=plan_start_s + (j+1)*dp_sample_s;
            double cur_node_l=((dp_row - 1)/2 -i)*dp_sample_l;
            if (std::abs(cur_node_l) >= 8){
                ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
            }
            for (int k=0;k<dp_row;k++){
                double pre_node_s=plan_start_s + j*dp_sample_s;
                double pre_node_l=((dp_row - 1)/2 -k)*dp_sample_l;
                if (std::abs(pre_node_l) >= 8){
                ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
                }
                double cost_neighbour=CalcNeighbourCost(pre_node_s,pre_node_l,
                        cur_node_s,cur_node_l,dp_cost_collision,dp_cost_smooth_dl,
                                dp_cost_smooth_ddl,dp_cost_smooth_dddl,dp_cost_ref,
                                static_obs_s_set,static_obs_l_set);
                double pre_min_cost=node_cost[k][j-1];
                double cost_temp=pre_min_cost + cost_neighbour;
                if (cost_temp < node_cost[i][j]){
                    node_cost[i][j]=cost_temp;
                    pre_node_index[i][j]=k;
                }
            }
        }
    }
    int index=0;
    double min_cost=std::numeric_limits<double>::max();
    for (int i=0;i<dp_row;i++){
        if (node_cost[i][dp_col-1] < min_cost){
            min_cost=node_cost[i][dp_col-1];
            index=i;
        }
    }

    vector<int> dp_node_list_row(dp_col,0);
    int cur_index=index;
    for (int i=0;i<dp_col;i++){
        int pre_index=pre_node_index[cur_index][dp_col-i-1];
        dp_node_list_row[dp_col-i-1]=cur_index;
        cur_index=pre_index;
    }

    vector<double> dp_path_s(15,-1.0);
    vector<double> dp_path_l(15,-1.0);
    for (int i=0;i<dp_col;i++){
        dp_path_s[i]=plan_start_s + (i+1)*dp_sample_s;
        dp_path_l[i]=((dp_row - 1)/2 -dp_node_list_row[i])*dp_sample_l;
        //std::cout<<"cur_ll=="<<dp_path_l[i]<<std::endl;
    }
    return std::make_tuple(dp_path_s,dp_path_l);
}

void Lattice::Increase_DP_Path_Node(const vector<double> &dp_path_s_init,
                        const vector<double> &dp_path_l_init,const double &plan_start_s,
                        const double &plan_start_l,const double &plan_start_dl,
                        const double &plan_start_ddl){
    double ds=1.0;
    for (int i=0;i<60;i++){
        dp_path_s_final[i]=std::nan("");
        dp_path_l_final[i]=std::nan("");
        dp_path_dl_final[i]=std::nan("");
        dp_path_ddl_final[i]=std::nan("");  
    }
    double start_s=plan_start_s;
    double start_l=plan_start_l;
    double start_dl=plan_start_dl;
    double start_ddl=plan_start_ddl;

    vector<double> s_cur;
    vector<double> l_temp;
    vector<double> dl_temp;
    vector<double> ddl_temp;
    for (int i=0;i<dp_path_s_init.size();i++){
        if (dp_path_s_init[i]==-1.0) break;

        for (int j=0;j<10000;j++){
            double s_node=start_s + ds*j;
            if (s_node < dp_path_s_init[i]){
                s_cur.push_back(s_node);
            }else{
                break;
            }
        }
        double end_s=dp_path_s_init[i];
        double end_l=dp_path_l_init[i];
        double end_dl=0.0;
        double end_ddl=0.0;
        Eigen::VectorXd coeff=CalcQuinticCoeffient(start_l, start_dl, start_ddl, end_l, 
                                                end_dl, end_ddl, start_s, end_s);
        double a0 = coeff(0);
        double a1 = coeff(1);
        double a2 = coeff(2);
        double a3 = coeff(3);
        double a4 = coeff(4);
        double a5 = coeff(5);

        int nn=s_cur.size();
        // 将 std::vector 转化为 Eigen::VectorXd
        //Eigen::VectorXd vecEigen = Eigen::Map<Eigen::VectorXd>(vecStd.data(), vecStd.size());
        Eigen::VectorXd ss_cur=Eigen::VectorXd::Zero(nn);
        for (int k=0;k<nn;k++){
            ss_cur[k]=s_cur[k];
        }
        Eigen::VectorXd l=Eigen::VectorXd::Zero(nn);
        Eigen::VectorXd dl=Eigen::VectorXd::Zero(nn);
        Eigen::VectorXd ddl=Eigen::VectorXd::Zero(nn);

        //Eigen::VectorXd result = vec.array().pow(3);
        l=a0 + a1*ss_cur.array() + a2*ss_cur.array().pow(2) + a3*ss_cur.array().pow(3) +
          a4*ss_cur.array().pow(4) + a5*ss_cur.array().pow(5);
        dl=a1 + 2*a2*ss_cur.array() + 3*a3*ss_cur.array().pow(2) + 4*a4*ss_cur.array().pow(3) +
           5*a5*ss_cur.array().pow(4);
        ddl=2 + 6*a3*ss_cur.array() + 12*a4*ss_cur.array().pow(2) +
            20*a5*ss_cur.array().pow(3);
        for (int k=0;k<nn;k++){
            l_temp.push_back(l(k));
            dl_temp.push_back(dl(k));
            ddl_temp.push_back(ddl(k));
        }

        start_s=end_s;
        start_l=end_l;
        start_dl=end_dl;
        start_ddl=end_ddl;

        s_cur.clear();
    }

    for (int uu=0;uu<l_temp.size();uu++){
        if (uu==60) break;
        dp_path_s_final[uu]=plan_start_s + ds*uu;
        dp_path_l_final[uu]=l_temp[uu];
        //std::cout<<"dp_path_l_final=="<<dp_path_l_final[uu]<<std::endl;
        dp_path_dl_final[uu]=dl_temp[uu];
        dp_path_ddl_final[uu]=ddl_temp[uu];
    }
}