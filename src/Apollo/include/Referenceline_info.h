#include "Global_Path.h"

using namespace std;
#ifndef REFERENCELINE_H
#define REFERENCELINE_H
#define PLAN_CYCLE 0.2
#define maxnum 128

struct Plan_Trajectory
{
    vector<double> trajectory_x;
    vector<double> trajectory_y;
    vector<double> trajectory_heading;
    vector<double> trajectory_kappa;
    vector<double> trajectory_velocity;
    vector<double> trajectory_accel;
    vector<double> trajectory_time;
};

struct Proj_Origin_Point
{
    int match_index;
    double proj_x;
    double proj_y;
    double proj_heading;
    double proj_kappa;
    Proj_Origin_Point (double x,double y,double head,double kap,int mat) : proj_x(x),
            proj_y(y), proj_heading(head), proj_kappa(kap), match_index(mat) {};
};

struct Obs_State
{
    double x;
    double y;
    double velocity;
    double heading;
};


struct Plan_Start_gcs
{
    double x;
    double y;
    double heading;
    double kappa;
    double vx;
    double vy;
    double ax;
    double ay;
    double start_time;
};

std::tuple<Plan_Start_gcs,Plan_Trajectory> Calc_Plan_Start_Info_And_Stitch_Trajectory(
                        const Plan_Trajectory &pre_trajectory, const double current_time,
                        const Host_State &host);
            


class FindHostOriginPoint{
public:
    double proj_x;
    double proj_y;
    double proj_heading;
    double proj_kappa;
    int pre_match_point_index_set=-1;

    FindHostOriginPoint()=default;
    void ImplementFind(const double host_x, const double host_y, 
    const vector<double> reference_x,const vector<double> reference_y, 
    const vector<double> reference_heading,const vector<double> reference_kappa);
private:
    

    
    
    bool first_run=true;
    

    vector<double> pre_reference_x;
    vector<double> pre_reference_y;
    vector<double> pre_reference_heading;
    vector<double> pre_reference_kappa;
};


class FindObsOriginPoint{
public:
    FindObsOriginPoint();
    void ImplementFind(const vector<double> obs_x, const vector<double> obs_y, 
    const vector<double> reference_x,const vector<double> reference_y, 
    const vector<double> reference_heading,const vector<double> reference_kappa);
    vector<Proj_Origin_Point> Get_Origin_Points();
private:
    

    vector<double> proj_x_set;
    vector<double> proj_y_set;
    vector<double> proj_heading_set;
    vector<double> proj_kappa_set;
    
    bool first_run;
    vector<int> pre_match_point_index_set;

    vector<double> pre_reference_x;
    vector<double> pre_reference_y;
    vector<double> pre_reference_heading;
    vector<double> pre_reference_kappa;

    vector<double> pre_x_set;
    vector<double> pre_y_set;
};

class Static_Obs{
public:
    
    void Init_Obs();
    void Handle_Obs(const Host_State &host,
                    const vector<Obs_State> &Obs, const vector<double> reference_x,
                    const vector<double> reference_y, const vector<double> reference_heading,
                    const vector<double> reference_kappa,const vector<double> &index2s);
    void Renew_Proj(const vector<double> &reference_x,const vector<double> &reference_y, 
                const vector<double> &reference_heading,const vector<double> &reference_kappa);
    void Renew_SL(const vector<double> &reference_x,const vector<double> &reference_y,
                            const vector<double> &index2s);
    std::tuple<vector<double>,vector<double>> Get_SL();
private:
    int num=32;
    vector<double> static_obs_s;
    vector<double> static_obs_l;
    vector<double> static_obs_x_set;
    vector<double> static_obs_y_set;
    vector<double> dynamic_obs_x_set;
    vector<double> dynamic_obs_y_set;
    vector<double> dynamic_obs_vx_set;
    vector<double> dynamic_obs_vy_set;

    

    vector<Proj_Origin_Point> proj_points;
    //FindObsOriginPoint *SeekSL;
    FindObsOriginPoint SeekObsOrigin;
};


vector<double> Get_Index2s(const vector<double> &reference_x,const vector<double> &reference_y,
        const double &origin_x, const double &origin_y, const int &match_index);

double CalcSFromIndex2S(const vector<double> &index2s,const vector<double> &path_x,
                        const vector<double> &path_y,const double &proj_x,const double &proj_y, 
                        const int &proj_match_point_index);

std::tuple<double,double,double,double> Cartesian2Frenet(const Plan_Start_gcs &plan_start, 
                const vector<double> &frenet_path_x,const vector<double> &frenet_path_y, 
                const Proj_Origin_Point &proj_point,const vector<double> &index2s);

std::tuple<vector<double>,vector<double>> Cartesian2Frenet(const vector<double> &x_set, 
                const vector<double> &y_set , const vector<double> &frenet_path_x,
            const vector<double> &frenet_path_y,const vector<Proj_Origin_Point> &proj_points,
            const vector<double> &index2s);

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>> Obs_Filter(
                        const Host_State &host,const vector<Obs_State> &Obs);

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>,
    vector<double>,vector<double>> OBS_Static_And_Dynamic(const vector<double> &obs_x_set,
            const vector<double> &obs_y_set,const vector<double> &obs_heading_set,
            const vector<double> &obs_velocity_set);

std::tuple<vector<double>,vector<double>> Get_Static_Obs_SL();



#endif