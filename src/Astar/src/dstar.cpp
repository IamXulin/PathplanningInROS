#include "dstar.h"




int main(int argc, char* argv[]){

    ros::init(argc,argv,"dstar");
    ros::NodeHandle nh;
    
    Node s_start(5,5);
    Node s_goal(45,25);
    DStar dstar;
    dstar.graphobs();
    ros::Rate(2).sleep();
    dstar.run(s_start, s_goal);
    
    std::cout<<"size="<<dstar.getpath().size()<<std::endl;

    while (ros::ok()){
        
        ros::spin();
        
        ros::Rate(10).sleep();
    }





    return 0;
}