#include "astar.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

double sx, sy, gx, gy;
bool sxy=false, gxy=false;
void dostart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    sx=std::round(msg->pose.pose.position.x);
    sy=std::round(msg->pose.pose.position.y);
    sxy=true;
}
void dogoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
    gx=std::round(msg->pose.position.x);
    gy=std::round(msg->pose.position.y);
    gxy=true;
}



int main(int argc, char* argv[]){

    ros::init(argc,argv,"astar");
    ros::NodeHandle nh;
    ros::Subscriber sub_start=nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,dogoal);
    ros::Subscriber sub_goal=nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10,dostart);
    
    AStar astar=AStar("euclidean");
    while (ros::ok()){
        if (sxy && gxy){
            Node start_node=Node(sx,sy);
            Node goal_node=Node(gx,gy);
            astar.AddNode(start_node, goal_node);
            std::vector<Node> path = astar.searching();
            astar.grapath();
            ros::Rate(10).sleep();
            return 0;
        }
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    

    return 0;
}