#include "emplanner.h"

int main(int argc, char *argv[]){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"kkkkkkkkk");
    ROS_INFO("Start Demo!");
    std::shared_ptr<Path_planning_node> pp=
                std::shared_ptr<Path_planning_node>(new Path_planning_node());
    if (pp->Init()){
        std::cout<<"planner is opticaling"<<std::endl;
    }else{
        return -1;
    }


    ros::spin();
    return 0;
}