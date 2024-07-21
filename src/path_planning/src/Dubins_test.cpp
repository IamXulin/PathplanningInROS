#include "Dubins_path.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double init_x,init_y,end_x,end_y;
double init_yaw,end_yaw;
/*
// 使用tf2库创建四元数
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);

    // 将四元数转换为geometry_msgs::Quaternion消息类型
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat);
*/
void pub_start_goal(double x,double y,double yaw,int id,ros::Publisher &pub,string choice){
    visualization_msgs::Marker pose;
    pose.header.frame_id="path";
    pose.header.stamp=ros::Time::now();
    pose.id=id;
    pose.type=visualization_msgs::Marker::ARROW;
    pose.action=visualization_msgs::Marker::ADD;
    pose.scale.x=3.0;
    pose.scale.y=1.0;
    pose.scale.z=1.0;
    pose.pose.position.x=x;
    pose.pose.position.y=y;
    pose.pose.position.z=0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0,0.0,yaw);
    // pose.pose.orientation.w=quat.getW();
    // pose.pose.orientation.x=quat.getX();
    // pose.pose.orientation.y=quat.getY();
    // pose.pose.orientation.z=quat.getZ();
    pose.pose.orientation = tf2::toMsg(quat);
    if (choice=="start"){
        pose.color.r=0.0;
        pose.color.g=1.0;
        pose.color.b=0.0;
        pose.color.a=1.0;
    }else if (choice=="goal"){
        pose.color.r=1.0;
        pose.color.g=0.0;
        pose.color.b=0.0;
        pose.color.a=1.0;
    }
    ros::Rate(5).sleep();
    pub.publish(pose);

}

void dostart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose){
    init_x=pose->pose.pose.position.x;
    init_y=pose->pose.pose.position.y;
    double roll,pitch;
    // 提取四元数信息
    tf2::Quaternion quat;
    tf2::fromMsg(pose->pose.pose.orientation,quat);
    // 将四元数转换为欧拉角
    tf2::Matrix3x3(quat).getRPY(roll,pitch,init_yaw);

}

void dogoal(const geometry_msgs::PoseStamped::ConstPtr &pose){
    end_x=pose->pose.position.x;
    end_y=pose->pose.position.y;
    double roll,pitch;
    // 提取四元数信息
    tf2::Quaternion quat;
    tf2::fromMsg(pose->pose.orientation,quat);
    // 将四元数转换为欧拉角
    tf2::Matrix3x3(quat).getRPY(roll,pitch,end_yaw);

}

int main(int argc, char *argv[]){

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"my_dubins");
    ros::NodeHandle nh;

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


    ros::Subscriber sub_start=nh.subscribe<geometry_msgs::
                    PoseWithCovarianceStamped>("/initialpose",10,dostart);
    ros::Subscriber sub_goal=nh.subscribe<geometry_msgs::
                    PoseStamped>("/move_base_simple/goal",10,dogoal);

    ros::Publisher pub_marker=nh.advertise<visualization_msgs::Marker>("/visualization_marker",10);
    // vector<vector<double>> states={{0,0,0},{10,10,-90},{20,5,60},{30,10,120},
    //                                 {35,-5,30},{25,-10,-120},{15,-15,100},{0,-10,-90}};
    ros::Rate r(5);
    while(ros::ok()){
        if (init_x){
            pub_start_goal(init_x,init_y,init_yaw,-1,pub_marker,"start");
        }
        if (end_x){
            pub_start_goal(end_x,end_y,end_yaw,-2,pub_marker,"goal");
        }
        if (init_x && end_x){
            vector<vector<double>> states={{init_x,init_y,init_yaw},{end_x,end_y,end_yaw}};
            Dubins_Path path=generate_path(states);
            for (int i=0;i<3;i++){
                r.sleep();
                visualization_path(path,pub_marker);
            }
            break;    
            
        }

        ros::spinOnce();
        
    }
    
    return 0;
}
















