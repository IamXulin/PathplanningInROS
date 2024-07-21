#include "Dijkstra.h"

int init_x,init_y,end_x,end_y;
void dostart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose){
    init_x=round(pose->pose.pose.position.x);
    init_y=round(pose->pose.pose.position.y);
}

void dogoal(const geometry_msgs::PoseStamped::ConstPtr &pose){
    end_x=round(pose->pose.position.x);
    end_y=round(pose->pose.position.y);
}



int main(int argc,char *argv[]){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"my_astar");
    ros::NodeHandle nh;
    DIJKSTRA dijkstra;
    dijkstra.sub_start=nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10,dostart);
    dijkstra.sub_goal=nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,dogoal);
    dijkstra.pub=nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",10);
    dijkstra.pub_SG=nh.advertise<visualization_msgs::Marker>("/visualization_marker",10);

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
    
    

    dijkstra.generate_obs();
    bool path_planning=false;
    bool motive_init=false;
    ros::Rate r(10);
    while(ros::ok()){
        dijkstra.pub_obs();
        if (init_x){
            dijkstra.Advertise_start(init_x,init_y);
        }
        if (end_x){
            dijkstra.Advertise_goal(end_x,end_y);
        }
        if (init_x && end_x && path_planning==false){
            dijkstra.planning();
            dijkstra.full_path();
            path_planning=true;
        }
        if (path_planning){
            dijkstra.pub.publish(dijkstra.Process);
            dijkstra.pub_SG.publish(dijkstra.PlotPath);
            break;
            }


        r.sleep();
        ros::spinOnce();
    }
    return 0;
}