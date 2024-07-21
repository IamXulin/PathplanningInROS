#include "Astar.h"

void Astar::Advertise_start(int init_x,int init_y){
    start_x=init_x;
    start_y=init_y;
    full_MarkerArray(start_x,start_y,-2,1.0,1.0,0.3,1.0,5.0,0.0,1.0,"start");
    pub_SG.publish(start);
}
void Astar::Advertise_goal(int end_x,int end_y){
    goal_x=end_x;
    goal_y=end_y;
    full_MarkerArray(goal_x,goal_y,-1,1.0,1.0,0.3,1.0,5.0,0.0,1.0,"goal");
    pub_SG.publish(goal);
}

void Astar::Advertise_process(){
    pub.publish(Process);
}

void Astar::full_PathArray(int x,int y){
    

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.3;
    PlotPath.points.push_back(p);


    
                
}



void Astar::full_MarkerArray(int x,int y,int id,double scale_x,double scale_y,
                            double scale_z,double color_r,
                double color_g,double color_b,double color_a,string choice){
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
    if (choice=="obs"){
        Obs_gather.markers.push_back(obs);
    }else if (choice=="start"){
        start=obs;
    }else if (choice=="goal"){
        goal=obs;
    }else if (choice=="process"){
        Process.markers.push_back(obs);
    }
    //else if (choice=="path"){
    //     ROS_INFO("ppppppppppppppppppp");
    //     PlotPath.markers.push_back(obs);
    // }

    
}

bool Astar::Iscollision(int next_x,int next_y){
    if (my_map[next_x][next_y]==1) return true;
    return false;
}

void Astar::generate_obs(){
    std::random_device seed;
    std::mt19937 prob(seed());
    std::uniform_int_distribution<> dospow(0,map_w-1);
    std::uniform_int_distribution<> dospoh(0,map_h-1);
    int id=0;
    for (int i=0;i<map_w;i++){
        for (int j=0;j<map_h;j++){
            double ranNum=static_cast<double> (dospow(prob))/(map_w-1);
            if (ranNum<0.3 || i==0 || i==map_w-1 || j==0 || j==map_h-1){
                my_map[i][j]=1;
                full_MarkerArray(i,j,id,0.5,0.5,0.0,1.0,0.0,0.0,1.0,"obs");
            }
            Nodelist[id]={i,j};
            Nodelistreverse[{i,j}]=id;
            g_cost[id]=10000;
            f_cost[id]=10000;
            id++;

        }
    }
}

void Astar::planning(){
    vector<int> start_node={start_x,start_y};
    vector<int> goal_node={goal_x,goal_y};
    int goal_id=Nodelistreverse[goal_node];
    Parentlist[start_node]=-1;
    int start_id=Nodelistreverse[start_node];
    ROS_INFO("起点ID为%.d",start_id);
    f_cost[start_id]=sqrt(pow(goal_x-start_x,2)+pow(goal_y-start_y,2));
    openlist.push_back(start_node);
    g_cost[start_id]=0;
    while (true){
        ros::Rate(100).sleep();
        int current_id;
        vector<int> current_node;
        double current_cost=1000000;
        //ROS_INFO("openlist长度==%d",openlist.size());
        for (int i=0;i<openlist.size();i++){
            vector<int> new_node=openlist[i];
            int new_id=Nodelistreverse[new_node];
            double new_cost=f_cost[new_id];
            if (new_cost<current_cost){
                current_cost=new_cost;
                current_id=new_id;
                current_node=new_node;
            }
        }
        
        //ROS_INFO("正在遍历点x==%.d   y==%.d  ID=%.d",current_node[0],current_node[1],current_id);
        if (sqrt(pow(goal_x-current_node[0],2)+pow(goal_y-current_node[1],2))<0.01){
            Parentlist[goal_node]=Parentlist[current_node];
            ROS_INFO("找到终点,他的ID为%d",goal_id);
            ROS_INFO("找到终点,他的父ID为%d",Parentlist[goal_node]);
            ROS_INFO("================================================================");
            break;
        }
        for (auto motion:motions){
            int next_x=current_node[0]+motion[0];
            int next_y=current_node[1]+motion[1];
            vector<int> next_node={next_x,next_y};
            int next_id=Nodelistreverse[next_node];
            if (Iscollision(next_x,next_y)){
                //ROS_INFO("3333333333");
                continue;
            }
            if (find(closedlist.begin(),closedlist.end(),next_node) !=closedlist.end()) continue;
            if (find(openlist.begin(),openlist.end(),next_node) !=openlist.end()){
                double oldcost=g_cost[next_id];
                double newcost=g_cost[current_id]+sqrt(pow(motion[0],2)+pow(motion[1],2));
                if (newcost<oldcost){
                    g_cost[next_id]=newcost;
                    f_cost[next_id]=g_cost[next_id]+sqrt(pow(goal_x-next_x,2)+pow(goal_y-next_y,2));
                    Parentlist[next_node]=current_id;
                    //ROS_INFO("%d=======>>>>%d",next_id,current_id);
                    
                }
            }else{
                g_cost[next_id]=g_cost[current_id]+sqrt(pow(motion[0],2)+pow(motion[1],2));
                f_cost[next_id]=g_cost[next_id]+sqrt(pow(goal_x-next_x,2)+pow(goal_y-next_y,2));
                Parentlist[next_node]=current_id;
                openlist.push_back(next_node);
                full_MarkerArray(next_x,next_y,next_id+1000,0.5,0.5,0.0,0.0,0.0,1.0,1.0,"process");
                
                Advertise_process();
                ROS_INFO("成功存入点x===%d y==%d",next_x,next_y);
                
                //ROS_INFO("成功存入点x===%d y==%d",next_x,next_y);
                //ROS_INFO("%d=======>>>>%d",next_id,current_id);
                //ROS_INFO("他的代价为%.2f",Costlist[next_id]);
            }
        }
        closedlist.push_back(current_node);
        auto iter=std::find(openlist.begin(),openlist.end(),current_node);
        if (iter != openlist.end()){
            openlist.erase(iter);
        }
        
    } 
    path.push_back(goal_node);
    int idd=Parentlist[goal_node];
    for (const auto &pair1:Nodelist){
        int u=pair1.first;
        if (u != Nodelistreverse[pair1.second]){
            ROS_INFO("错误!!!!!");
        }
                
    }
    //ros::Rate p(1);
    while (idd!=-1){
        vector<int> nodee=Nodelist[idd];
        //ROS_INFO("当前点ID%.d",idd);
        path.push_back(nodee);
        //ROS_INFO("当前点的父节点ID%.d",Parentlist[nodee]);
        if (sqrt(pow(start_node[0]-nodee[0],2)+pow(start_node[1]-nodee[1],2))<0.001) break;
        idd=Parentlist[nodee];
        //p.sleep();
    }

}

void Astar::pub_obs(){
    pub.publish(Obs_gather);
    
}


// void Astar::full_path(){
//     int id=-500;
//     for (int i=0;i<path.size();i++){
//         full_MarkerArray(path[i][0],path[i][1],id,0.5,0.5,0.2,0.0,1.0,0.0,1.0,"path");
//         id++;
//     }
    
// }
void Astar::full_path(){
    PlotPath.header.frame_id = "path";
    PlotPath.header.stamp = ros::Time::now();
    PlotPath.ns = "line";
    PlotPath.id = 0;
    PlotPath.type = visualization_msgs::Marker::LINE_STRIP;
    PlotPath.action = visualization_msgs::Marker::ADD;
    PlotPath.pose.orientation.w = 1.0;
    PlotPath.scale.x = 0.5;  // Line width
    PlotPath.scale.z=0.5;
    for (int i=0;i<path.size();i++){
        full_PathArray(path[i][0],path[i][1]);
    }
    PlotPath.color.r = 0.0;
    PlotPath.color.g = 1.0;
    PlotPath.color.b = 0.0;
    PlotPath.color.a = 1.0;
    ROS_INFO("888888888888888888888888888888");
    
}

