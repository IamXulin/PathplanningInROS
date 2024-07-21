#include "LQR_Control.h"
using namespace std;

LQRController::LQRController(){
    
    A.resize(Nx,Nx);
    B.resize(Nx,Nu);
    Q=Eigen::MatrixXd::Identity(Nx,Nx)*100;
    R=Eigen::MatrixXd::Identity(Nu,Nu)*5;
}
void LQRController::Init_Parameters(double x, double y, double yawr, double v, double kappar){
    xr=x;
    yr=y;
    std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
    thetar=yawr;
    vr=v;
    delta_r=atan(host_L*kappar);
}
void LQRController::linerization(){
    A << 1,   0,    -dt*vr*sin(thetar),
           0,   1,     dt*vr*cos(thetar),
           0,   0,          1;

    B <<  dt*cos(thetar), 0,
            dt*sin(thetar), 0,
            dt*tan(delta_r)/host_L, dt*vr/(host_L*pow(cos(delta_r),2));
}

Eigen::MatrixXd LQRController::claculate_P(){
    Eigen::MatrixXd Pn=Q;
    Eigen::MatrixXd P_=Q;
    for (int i=0;i<maxiter;i++){
        P_=Q + A.transpose()*Pn*A -A.transpose()*Pn*B*(R+B.transpose()*Pn*B).inverse()
                        *B.transpose()*Pn*A;
        if ((P_-Pn).norm()<0.2) break;
        Pn=P_;
    }
    return P_;
}

std::tuple<double,double> LQRController::solveLQR(const double &host_x, 
                        const double &host_y, const double &host_yaw){
    linerization();
    Eigen::Vector3d X;
    X << host_x-xr, host_y-yr, host_yaw-thetar;
    if (X(2) > M_PI){
        X(2) -= 2*M_PI;
    }else if (X(2) < -M_PI){
        X(2) += 2*M_PI;
    }
    Eigen::MatrixXd P=claculate_P();
    Eigen::MatrixXd K=-(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
    Eigen::Vector2d OUT=K*X;

    return std::make_tuple(vr+OUT(0) , delta_r+OUT(1));

}