#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <Eigen/Dense>

class LQRController{
public:
    LQRController();
    void Init_Parameters(double x, double y, double yawr, double v, double kappar);
    void linerization();
    std::tuple<double,double> solveLQR(const double &host_x, 
                        const double &host_y, const double &host_yaw);
    Eigen::MatrixXd claculate_P();
private:
    double xr;
    double yr;
    double thetar;
    double vr;
    double delta_r;

    const int Nx=3;
    const int Nu=2;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    

    
    double host_L=3.0;
    double dt=1.0/50.0;
    int maxiter=200;
};