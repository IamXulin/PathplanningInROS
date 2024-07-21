#include "DWA.h"

visualization_msgs::Marker sstart;
visualization_msgs::Marker ggoal;
int init_x,init_y,end_x,end_y;
void dostart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose){
    init_x=round(pose->pose.pose.position.x);
    init_y=round(pose->pose.pose.position.y);
}

void dogoal(const geometry_msgs::PoseStamped::ConstPtr &pose){
    end_x=round(pose->pose.position.x);
    end_y=round(pose->pose.position.y);
}

void full_MarkerArray(int x,int y,int id,double scale_x,double scale_y,
                        double scale_z,double color_r,double color_g,
                        double color_b,double color_a,string choice){
    visualization_msgs::Marker obs;
    obs.header.stamp = ros::Time::now();
    obs.header.frame_id="path";
    // 设置 ID
    obs.id = id;
    // 设置类型为立方体
    obs.type = visualization_msgs::Marker::CUBE;
    // 设置尺寸
    obs.scale.x = scale_x;
    obs.scale.y = scale_y;
    obs.scale.z = scale_z;
    // 设置颜色，这里为红色
    obs.color.r = color_r;
    obs.color.g = color_g;
    obs.color.b = color_b;
    obs.color.a = color_a; // 不透明
    // 设置位置
    obs.pose.position.x = x;
    obs.pose.position.y = y;
    obs.pose.position.z = 0.0;
    obs.pose.orientation.x=0.0;
    obs.pose.orientation.y=0.0;
    obs.pose.orientation.z=0.0;
    obs.pose.orientation.w=0.0;
    if (choice=="start"){
        sstart=obs;
    }else if (choice=="goal"){
        ggoal=obs;
    }
}

int main(int argc,char *argv[]){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"my_dwa");
    ros::NodeHandle nh;
    ros::Subscriber sub_start=nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10,dostart);
    ros::Subscriber sub_goal=nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,dogoal);
    ros::Publisher pub=nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",10);
    ros::Publisher pub_SG=nh.advertise<visualization_msgs::Marker>("/visualization_marker",10);

    
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
    

    DWA dwa(nh);
    bool motive_init=false;
    
    ros::Rate r(10);
    while(ros::ok()){
        if (init_x){
            full_MarkerArray(init_x,init_y,-2,1.0,1.0,0.3,1.0,5.0,0.0,1.0,"start");
            pub_SG.publish(sstart);
            dwa.Advertise_start(init_x,init_y);
        }
        if (end_x){
            full_MarkerArray(end_x,end_y,-2,1.0,1.0,0.3,1.0,5.0,0.0,1.0,"goal");
            pub_SG.publish(ggoal);
            dwa.Advertise_goal(end_x,end_y);
        }

        while(init_x && end_x){
            
            //ROS_INFO("机器人位置x==%.2f   y==%.2f",dwa.ego_state.x,dwa.ego_state.y);
            double distance=sqrt(pow(end_x-dwa.ego_state.x,2)+pow(end_y-dwa.ego_state.y,2));
            //ROS_INFO("distance==%.2f",distance);
            dwa.trajectory_evaluation();
            //ROS_INFO("end_x==%.d",end_x);
            if (distance<0.5){
                dwa.ego_control.v=0;
                dwa.ego_control.w=0;
                dwa.ego_state.V=0;
                dwa.ego_state.W=0;
                ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                return 0;
            }
            dwa.state_update();
            //astar.pub_SG.publish(astar.PlotPath);
            r.sleep();
            
        }
        r.sleep();
        ros::spinOnce();
    }
    
    return 0;
}