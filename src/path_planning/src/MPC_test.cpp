#include"MPC.h"


int main(int argc,char *argv[]){

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"my_mpc");
    ros::NodeHandle nh;
    ros::Publisher pu=nh.advertise<visualization_msgs::Marker>("visualization_marker",10);
    ros::Publisher pu_err=nh.advertise<std_msgs::Float64>("/laterr",10);
    
    ros::Publisher marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Publisher marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id="path";
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    // Clear marker array
    visualization_msgs::MarkerArray clear_marker_array;
    clear_marker_array.markers.push_back(clear_marker);
    ros::Rate(2).sleep();
    marker_pub_.publish(clear_marker);
    marker_array_pub_.publish(clear_marker_array);

    MPC mpc(pu, pu_err);
    //ros::Rate r(200);
    //Eigen::MatrixXd X=Eigen::MatrixXd::Zero(3,3);
    //X.block(0,0,3,3)=Eigen::MatrixXd::Identity(3,3);
    //cout<< X <<endl;
    while(ros::ok()){
        mpc.pub_path();
        mpc.solveQP();
        //r.sleep();
    }









    return 0;
}

