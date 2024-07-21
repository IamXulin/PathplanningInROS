#include "Referenceline_info.h"
using namespace std;

#ifndef LATTICE_H
#define LATTICE_H



class Lattice
{
private:
    double dp_cost_collision;
    double dp_cost_smooth_dl;
    double dp_cost_smooth_ddl;
    double dp_cost_smooth_dddl;
    double dp_cost_ref;
    int dp_row;
    int dp_col;
    double dp_sample_s;
    double dp_sample_l;

    vector<double> dp_path_s_final;
    vector<double> dp_path_l_final;
    vector<double> dp_path_dl_final;
    vector<double> dp_path_ddl_final;
public:
    Lattice(double cost_collision,double cost_dl,double cost_ddl,double cost_dddl,
                    double cost_ref,int row,int col,int sample_s,int sample_l);
    std::tuple<vector<double>,vector<double>,vector<double>,vector<double>> Get_SLDLDDL();
    std::tuple<vector<double>,vector<double>> Dynamic_Planning(
        const vector<double> &static_obs_s_set,const vector<double> &static_obs_l_set,
        const double &plan_start_s,const double &plan_start_l,const double &plan_start_dl,
        const double &plan_start_ddl);
    void Increase_DP_Path_Node(const vector<double> &dp_path_s_init,
                        const vector<double> &dp_path_l_init,const double &plan_start_s,
                        const double &plan_start_l,const double &plan_start_dl,
                        const double &plan_start_ddl);   
};

double CalcObsCost(double w_cost_collision,Eigen::VectorXd &square_d);

double CalcStartCost(const double &begin_s,const double &begin_l,
        const double &begin_dl,const double &begin_ddl,int cur_node_row,
        double sample_s,double sample_l,double w_cost_collision,double w_cost_smooth_dl,
        double w_cost_smooth_ddl,double w_cost_cost_dddl,double w_cost_ref,
        const vector<double> &obs_s_set,const vector<double> &obs_l_set,int row);

Eigen::VectorXd CalcQuinticCoeffient(double start_l,double start_dl,double start_ddl,
            double end_l,double end_dl,double end_ddl,double start_s,double end_s);

double CalcNeighbourCost(double pre_node_s,double pre_node_l,double cur_node_s,
                        double cur_node_l,double dp_cost_collision,double dp_cost_smooth_dl,
                double dp_cost_smooth_ddl,double dp_cost_smooth_dddl,double dp_cost_ref,
            const vector<double> &static_obs_s_set,const vector<double> &static_obs_l_set);

#endif