#include "MPC.h"
#ifndef LQR_H
#define LQR_H

using namespace std;


class LQR{
public:
    LQR(ros::Publisher pub1, ros::Publisher pub2);
    void curve_generate();
    void referencepath_generate();
    void init_ego();
    Position ego_position;
    vector<geometry_msgs::Point> point_gather;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker ego;
    int indexx=0;
    vector<reference_Node> reference_path;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    vector<double> cal_target_reference(double x,double y);
    void linerization(const double &phi_r, const double &v_r,const double &delta_r);
    void update(vector<double> &out);
    void solveLQR();
    void pub_path();
    Eigen::MatrixXd claculate_P();

private:
    ros::Publisher pub;
    ros::Publisher pub_err;
    const int Nx=3;
    const int Nu=2;
    double L=2.0;
    double dt=0.1;
    int maxiter=200;


};



#endif