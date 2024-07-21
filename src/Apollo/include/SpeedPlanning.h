#include "Referenceline_info.h"

#ifndef SPEED_H
#define SPEED_H

double interpolate(double x_interp, const vector<double> &x_values,
                                const vector<double> &y_values);

std::tuple<double,vector<double>> Calc_Path_Length_And_Map_Between_Point_And_S(
            const vector<double> &trajectory_x, const vector<double> &trajectory_y);

std::tuple<double,double> Calc_Speed_Planning_Start_Condition(const double &plan_start_vx,
                    const double &plan_start_vy, const double &plan_start_ax,
                    const double &plan_start_ay, const double &plan_start_heading);

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>> Sample_Speed_Qp_Planning(const double &plan_start_s_dot,
            const double &plan_start_s_dot2, const double &s_end, const double &recomment_T);

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>> Increase_St_Point_Count(
    const vector<double> &s_init, const vector<double> &s_dot_init, 
    const vector<double> &s_dot2_init, const vector<double> &relative_time_init);

Plan_Trajectory Path_And_Speed_Merge(const vector<double> &s, const vector<double> &s_dot, 
                    const vector<double> &s_dot2, const vector<double> &relative_time, 
                    const double &current_time, const vector<double> &path_s, 
                    const vector<double> &trajectory_x_init, const vector<double> &trajectory_y_init,
                    const vector<double> &trajectory_heading_init, const vector<double> &trajectory_kappa_init);

Plan_Trajectory Stitch_Trajectory(const Plan_Trajectory &trajectory, const Plan_Trajectory &stitch);


#endif
