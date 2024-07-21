#include "Lattice.h"

using namespace std;
#ifndef PATH_QP_H
#define PATH_QP_H

class PATH_QP_Solver{
public:
    PATH_QP_Solver(double cost_l,double cost_dl,double cost_ddl,double cost_dddl,
                double cost_centre,double cost_end_l,double cost_end_dl,double cost_end_ddl,
                double d1,double d2,double w);
    void Generate_Convex_Space(const vector<double> &dp_path_s,
                    const vector<double> &dp_path_l,const vector<double> &static_obs_s_set,
                    const vector<double> &static_obs_l_set);
    void Path_Planning_With_Quadratic_Programming(double plan_start_s,double plan_start_l,
                                    double plan_start_dl,double plan_start_ddl);
    void Increase_QP_Node();
    std::tuple<vector<double>,vector<double>,vector<double>,vector<double>> Get_QP_Result();
private:
    double w_cost_l;
    double w_cost_dl;
    double w_cost_ddl;
    double w_cost_dddl;
    double w_cost_centre;
    double w_cost_end_l;
    double w_cost_end_dl;
    double w_cost_end_ddl;
    double host_d1;
    double host_d2;
    double host_w;

    double static_obs_length=2.0;
    double static_obs_width=1.0;

    double delta_dl_max=0.1;
    double delta_ddl_max=0.05;

    vector<double> l_min;
    vector<double> l_max;

    vector<double> qp_path_s;
    vector<double> qp_path_l;
    vector<double> qp_path_dl;
    vector<double> qp_path_ddl;

    vector<double> qp_path_s_final;
    vector<double> qp_path_l_final;
    vector<double> qp_path_dl_final;
    vector<double> qp_path_ddl_final;
};

int FindNearIndex(const vector<double> &dp_path_s, double obs_s);

#endif