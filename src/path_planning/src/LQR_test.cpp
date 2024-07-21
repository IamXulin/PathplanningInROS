#include"LQR.h"


int main(int argc,char *argv[]){

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"my_lqr");
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


    LQR lqr(pu, pu_err);
    //ros::Rate r(10);
    while(ros::ok()){
        lqr.pub_path();
        lqr.solveLQR();
        //r.sleep();
    }
}