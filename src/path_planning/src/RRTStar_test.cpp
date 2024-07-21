#include "RRTStar.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

double init_x,init_y,end_x,end_y;
visualization_msgs::Marker start;
visualization_msgs::Marker goal;
void visualization_start_goal(double x, double y, int id, string choic){
    visualization_msgs::Marker Pose;
    Pose.header.frame_id="path";
    Pose.header.stamp=ros::Time::now();
    Pose.id=id;
    Pose.type=visualization_msgs::Marker::CUBE;
    Pose.action=visualization_msgs::Marker::ADD;
    Pose.scale.x=1.0;
    Pose.scale.y=1.0;
    Pose.scale.z=0.0;
    Pose.pose.position.x=x;
    Pose.pose.position.y=y;
    Pose.pose.position.z=0.0;
    Pose.color.r=1.0;
    Pose.color.g=5.0;
    Pose.color.b=0.0;
    Pose.color.a=1.0;
    if (choic=="start"){
        start=Pose;
    }else if (choic=="goal"){
        goal=Pose;
    }
}
void dostart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose){
    init_x=pose->pose.pose.position.x;
    init_y=pose->pose.pose.position.y;
    ROS_INFO("起点x==%.2f  y==%.2f",init_x,init_y);
}

void dogoal(const geometry_msgs::PoseStamped::ConstPtr &pose){
    end_x=pose->pose.position.x;
    end_y=pose->pose.position.y;
    ROS_INFO("终点x==%.2f  y==%.2f",end_x,end_y);
}

int main(int argc, char *argv[]){

    setlocale(LC_ALL,"");

    ros::init(argc,argv,"my_rrt");
    ros::NodeHandle nh;
    
    ros::Subscriber sub_start=nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10,dostart);
    ros::Subscriber sub_goal=nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,dogoal);
    ros::Publisher pub=nh.advertise<visualization_msgs::Marker>("/visualization_marker",10);

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
    

    RRT rrt(nh);

    bool gx=false;
    bool gy=false;
    
    while(ros::ok()){
        if (init_x && gx==false){
            gx=true;
            visualization_start_goal(init_x, init_y,1000,"start");
            pub.publish(start);
            rrt.init_start(start);
        }
        if (end_x && gy==false){
            gy=true;
            visualization_start_goal(end_x, end_y,1001,"goal");
            pub.publish(goal);
            rrt.init_goal(goal);
        }
        if (gx && gy){
            rrt.planning();
            break;
        }

        ros::spinOnce();
    }


    return 0;
}