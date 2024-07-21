#include "Global_Path.h"


int main (int argc, char *argv[]){

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"global");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<visualization_msgs::Marker>
                            ("visualization_marker",100000);

    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id="path";
    line_marker.header.stamp=ros::Time::now();
    //line_marker.ns="line";
    line_marker.id=1000;
    line_marker.type=visualization_msgs::Marker::LINE_STRIP;
    line_marker.action=visualization_msgs::Marker::ADD;
    line_marker.scale.x=0.1;
    line_marker.color.r=1.0;
    line_marker.color.g=0.0;
    line_marker.color.b=0.0;
    line_marker.color.a=1.0;
    geometry_msgs::Point p;
    auto globalpath=generate_globalpath();
    vector<double> global_x=std::get<0>(globalpath);
    vector<double> global_y=std::get<1>(globalpath);
    auto heading_kappa=Cal_heading_kappa(global_x,global_y);
    vector<double> global_heading=std::get<0>(heading_kappa);
    vector<double> global_kappa=std::get<1>(heading_kappa);
    //vector<GlobalNode> gg=QP_globalpath(globalpath);
    double ex=0;
    double ey=0;
    int mm=referenceline_provider_seek_match_index(ex,ey,global_x,global_y,global_heading,global_kappa);
    auto vv=UnSmoothPath(mm,global_x,global_y);
    vector<double> reference_x_init=std::get<0>(vv);
    vector<double> reference_y_init=std::get<1>(vv);
    auto smooth_path=Smooth_Globalpath(reference_x_init,reference_y_init);
    vector<double> reference_x=std::get<0>(smooth_path);
    vector<double> reference_y=std::get<1>(smooth_path);
    
    for (int i=0;i<reference_x.size();i++){
        p.x=reference_x[i];
        p.y=reference_y[i];
        p.z=0.0;
        //ros::Rate(50).sleep();
        //std::cout<<"head=="<<node.heading<<"    kappa=="<<node.kappa<<std::endl;
        line_marker.points.push_back(p);
    }
    
    ros::Rate r(1);
    while(ros::ok()){
        pub.publish(line_marker);
        //ROS_INFO("xx==%.2f",globalpath[globalpath.size()-1]);
        ROS_INFO("发布成功!!!");

        r.sleep();
    }
    
    return 0;
}