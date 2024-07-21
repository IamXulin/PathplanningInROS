#include "RRTStar.h"




vector<vector<double>> get_ray(Node &start, Node &end){
    vector<double> orig={start.x , start.y};
    vector<double> direc={end.x-start.x , end.y-start.y};
    vector<vector<double>> return_={orig,direc};
    return return_;
}

double get_dist(Node &start, Node &end){
    double dis=sqrt(pow(end.x-start.x,2)+pow(end.y-start.y,2));
    return dis;
}



void RRT::generate_obs(){
    obs_circle={{7,12,3},{46,20,2},{15,5,2},{37,7,3},{37,23,3}};
    obs_rectangle={{14,12,8,2},{18,22,8,3},{26,7,2,12},{32,14,10,2}};
    obs_boundary={{0,0,1,30},{0,30,50,1},{1,0,50,1},{50,1,1,30}};
}

RRT::RRT(ros::NodeHandle &nh){
    pubarry=nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",10);
    pubmark=nh.advertise<visualization_msgs::Marker>("/visualization_marker",10);
    //vertex.push_back(start);
    generate_obs();
    visualization_obs();
}

void RRT::init_start(visualization_msgs::Marker &pose){
    start.x=pose.pose.position.x;
    start.y=pose.pose.position.y;
    start.id=-15001;
    start.parent=-15002;
    start.cost=0.0;
    vertex.push_back(start);
    Node_gather[start.id]=start;
    
}
void RRT::init_goal(visualization_msgs::Marker &pose){
    goal.x=pose.pose.position.x;
    goal.y=pose.pose.position.y;
    goal.id=1;
    goal.parent=1;
    //vertex.push_back(goal);
    Node_gather[goal.id]=goal;
}

void RRT::visualization_vertex(Node &start, Node &end, int id){
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id="path";
    line_marker.header.stamp=ros::Time::now();
    line_marker.ns="line";
    line_marker.id=id;
    line_marker.type=visualization_msgs::Marker::LINE_STRIP;
    line_marker.action=visualization_msgs::Marker::ADD;
    line_marker.scale.x=0.1;
    line_marker.color.r=1.0;
    line_marker.color.g=5.0;
    line_marker.color.b=0.0;
    line_marker.color.a=0.5;
    geometry_msgs::Point p1,p2;
    p1.x=start.x; p1.y=start.y; p1.z=0.0;
    p2.x=end.x; p2.y=end.y; p2.z=0.0;
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);

    ros::Rate(200).sleep();
    pubmark.publish(line_marker);
}

void RRT::revisualization_vertex(Node &start, Node &end, int id){
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id="path";
    line_marker.header.stamp=ros::Time::now();
    line_marker.ns="line";
    line_marker.id=id;
    line_marker.type=visualization_msgs::Marker::LINE_STRIP;
    line_marker.action=visualization_msgs::Marker::ADD;
    line_marker.scale.x=0.1;
    line_marker.color.r=1.0;
    line_marker.color.g=5.0;
    line_marker.color.b=0.0;
    line_marker.color.a=0.5;
    geometry_msgs::Point p1,p2;
    p1.x=start.x; p1.y=start.y; p1.z=0.0;
    p2.x=end.x; p2.y=end.y; p2.z=0.0;
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);

    ros::Rate(200).sleep();
    pubmark.publish(line_marker);
}

void RRT::visualization_obs(){
    int id=0;
    for (int i=0;i<obs_rectangle.size();i++){
        visualization_msgs::Marker rectangle_obs;
        rectangle_obs.header.frame_id="path";
        rectangle_obs.id=id;
        rectangle_obs.header.stamp=ros::Time::now();
        rectangle_obs.type=visualization_msgs::Marker::CUBE;
        rectangle_obs.action=visualization_msgs::Marker::ADD;
        rectangle_obs.pose.position.x=obs_rectangle[i].x+obs_rectangle[i].len/2;
        rectangle_obs.pose.position.y=obs_rectangle[i].y+obs_rectangle[i].h/2;
        rectangle_obs.pose.position.z=0.0;
        rectangle_obs.scale.x=obs_rectangle[i].len;
        rectangle_obs.scale.y=obs_rectangle[i].h;
        rectangle_obs.color.r=1.0;
        rectangle_obs.color.g=0.0;
        rectangle_obs.color.b=0.0;
        rectangle_obs.color.a=1.0;
        Obs_gather.markers.push_back(rectangle_obs);
        id++;
    }
    for (int j=0;j<obs_boundary.size();j++){
        visualization_msgs::Marker rectangle_obs;
        rectangle_obs.header.frame_id="path";
        rectangle_obs.id=id;
        rectangle_obs.header.stamp=ros::Time::now();
        rectangle_obs.type=visualization_msgs::Marker::CUBE;
        rectangle_obs.action=visualization_msgs::Marker::ADD;
        rectangle_obs.pose.position.x=obs_boundary[j].x+obs_boundary[j].len/2;
        rectangle_obs.pose.position.y=obs_boundary[j].y+obs_boundary[j].h/2;
        rectangle_obs.pose.position.z=0.0;
        rectangle_obs.scale.x=obs_boundary[j].len;
        rectangle_obs.scale.y=obs_boundary[j].h;
        rectangle_obs.color.r=1.0;
        rectangle_obs.color.g=0.0;
        rectangle_obs.color.b=0.0;
        rectangle_obs.color.a=1.0;
        Obs_gather.markers.push_back(rectangle_obs);
        id++;
    }
    for (int k=0;k<obs_circle.size();k++){
        visualization_msgs::Marker circle_obs;
        circle_obs.header.frame_id="path";
        circle_obs.header.stamp=ros::Time::now();
        circle_obs.id=id;
        circle_obs.type=visualization_msgs::Marker::SPHERE;
        circle_obs.action=visualization_msgs::Marker::ADD;
        circle_obs.pose.position.x=obs_circle[k].x;
        circle_obs.pose.position.y=obs_circle[k].y;
        circle_obs.pose.position.z=0.0;
        circle_obs.scale.x=2*obs_circle[k].radius;
        circle_obs.scale.y=2*obs_circle[k].radius;
        circle_obs.scale.z=0.0;
        circle_obs.color.r=1.0;
        circle_obs.color.g=0.0;
        circle_obs.color.b=0.0;
        circle_obs.color.a=1.0;
        Obs_gather.markers.push_back(circle_obs);
        id++;
    }

    for (int i=0;i<5;i++){
        ros::Rate(5).sleep();
        pubarry.publish(Obs_gather);
    }
    
}

bool RRT::is_inside_obs(Node &node){
    for (int i=0; i<obs_circle.size(); i++){
        double x=obs_circle[i].x;
        double y=obs_circle[i].y;
        double r=obs_circle[i].radius;
        if (sqrt(pow(node.x-x,2)+pow(node.y-y,2)) <= r+delta){
            return true;
        }
    }
    for (int j=0; j<obs_rectangle.size();j++){
        double x=obs_rectangle[j].x;
        double y=obs_rectangle[j].y;
        double w=obs_rectangle[j].len;
        double h=obs_rectangle[j].h;
        if (node.x-(x-delta)>=0 && node.x-(x-delta)<=w+2*delta && 
            node.y-(y-delta)>=0 && node.y-(y-delta)<=h+2*delta){
                return true;
        }
    }
    for (int k=0; k<obs_boundary.size();k++){
        double x=obs_boundary[k].x;
        double y=obs_boundary[k].y;
        double w=obs_boundary[k].len;
        double h=obs_boundary[k].h;
        if (node.x-(x-delta)>=0 && node.x-(x-delta)<=w+2*delta && 
            node.y-(y-delta)>=0 && node.y-(y-delta)<=h+2*delta){
                return true;
        }
    }
    return false;

}

bool RRT::is_intersect_rec(Node &start, Node &end, vector<double> &org,
                          vector<double> &direc, vector<double> &a, vector<double> &b){
    vector<double> v1={org[0]-a[0],org[1]-a[1]};
    vector<double> v2={b[0]-a[0],b[1]-a[1]};
    vector<double> v3={-direc[1],direc[0]};
    
    double div=v2[0]*v3[0]+v2[1]*v3[1];

    if (div==0) return false;

    double t1=(v2[0]*v1[1]-v2[1]*v1[0])/div;
    double t2=(v1[0]*v3[0]+v1[1]*v3[1])/div;

    if (t1>=0 && t2>=0 && t2<=1){
        Node shot={org[0]+t1*direc[0], org[1]+t1*direc[1]};
        double dist_obs=get_dist(start, shot);
        double dist_seg=get_dist(start,end);
        if (dist_obs <= dist_seg) return true;
    }
    return false;
}

vector<vector<vector<double>>> RRT::get_obs_vertex(){
    vector<vector<vector<double>>> obs_list;
    for (int i=0;i<obs_rectangle.size();i++){
        double ox=obs_rectangle[i].x;
        double oy=obs_rectangle[i].y;
        double w=obs_rectangle[i].len;
        double h=obs_rectangle[i].h;
        vector<vector<double>> vertex_list={{ox-delta,oy-delta},{ox+w+delta,oy-delta},
                                            {ox+w+delta,oy+h+delta},{ox-delta,oy+h+delta}};
        obs_list.push_back(vertex_list);
    }
    return obs_list;

}

bool RRT::is_intersect_circle(vector<double> org, vector<double> direc, 
                             vector<double> a, double r){
    Eigen::Vector2d d(direc[0],direc[1]);
    double d2=d.dot(d);
    if (d2==0) return false;
    Eigen::Vector2d ao(a[0]-org[0],a[1]-org[1]);
    double t=ao.dot(d)/d2;
    if (t>=0 && t<=1){
        Node shot={org[0]+t*direc[0],org[1]+t*direc[1]};
        Node aa={a[0],a[1]};
        if (get_dist(shot,aa)<= r+delta) return true;
    }
    return false;
}


bool RRT::is_collision(Node &start, Node &end){
    if (is_inside_obs(start) || is_inside_obs(end)) return true;

    vector<vector<double>> od=get_ray(start,end);
    vector<double> org=od[0];
    vector<double> direc=od[1];

    vector<vector<vector<double>>> obs_list=get_obs_vertex();
    for (int i=0;i<obs_list.size();i++){
        vector<double> v1=obs_list[i][0];
        vector<double> v2=obs_list[i][1];
        vector<double> v3=obs_list[i][2];
        vector<double> v4=obs_list[i][3];

        if (is_intersect_rec(start,end,org,direc,v1,v2)) return true;

        if (is_intersect_rec(start,end,org,direc,v2,v3)) return true;

        if (is_intersect_rec(start,end,org,direc,v3,v4)) return true;

        if (is_intersect_rec(start,end,org,direc,v4,v1)) return true;
    }

    for (int j=0;j<obs_circle.size();j++){
        double x=obs_circle[j].x;
        double y=obs_circle[j].y;
        double r=obs_circle[j].radius;
        vector<double> a={x,y};
        /*
        is_intersect_circle(std::vector<double> &org, 
        std::vector<double> &direc, std::vector<double> &a, double &r)
        */
        if (is_intersect_circle(org,direc,a,r)) return true;
    }
    return false;

}

vector<double> RRT::get_distance_and_angle(Node &node_start, Node &node_end){
    double dx=node_end.x-node_start.x;
    double dy=node_end.y-node_start.y;
    double distance=sqrt(pow(dx,2)+pow(dy,2));
    double angle=atan2(dy,dx);
    vector<double> distance_and_angle={distance,angle};
    return distance_and_angle;
}

Node RRT::nearest_neighbor(Node &node_rand){
    double G=100000000;
    int ii;
    
    for (int i=0;i<vertex.size();i++){
        if (sqrt(pow(vertex[i].x-node_rand.x,2)+pow(vertex[i].y-node_rand.y,2))<G){
            ii=i;
            G=sqrt(pow(vertex[i].x-node_rand.x,2)+pow(vertex[i].y-node_rand.y,2));
        }
    }
    
    return vertex[ii];
}

Node RRT::new_state(Node &node_start, Node &node_end, int id){
    vector<double> distance_and_angle=get_distance_and_angle(node_start,node_end);
    double dist=distance_and_angle[0];
    double theta=distance_and_angle[1];

    dist=min(step_len,dist);
    Node node_new={node_start.x+dist*cos(theta),node_start.y+dist*sin(theta)};
    node_new.parent=node_start.id;
    node_new.id=id;
    node_new.cost=get_new_cost(node_start, node_new);
    return node_new;
}

void RRT::extract_path(Node &node_end){
    path.push_back({goal.x,goal.y});
    Node node_now=node_end;
    while (node_now.parent != -15002){
        path.push_back({node_now.x,node_now.y});
        node_now = Node_gather[node_now.parent];
        if (node_now.x==start.x && node_now.y==start.y) break;
        //ros::Rate(1).sleep();
    }
    path.push_back({node_now.x,node_now.y});
}

void RRT::visualization_path(){
    visualization_msgs::Marker Path;
    Path.header.frame_id="path";
    Path.header.stamp=ros::Time::now();
    Path.id=999;
    Path.ns="path";
    Path.type=visualization_msgs::Marker::LINE_STRIP;
    Path.action=visualization_msgs::Marker::ADD;
    Path.scale.x=0.3;
    Path.scale.z=0.4;
    Path.color.r=0.0;
    Path.color.g=1.0;
    Path.color.b=0.0;
    Path.color.a=1.0;
    //vector<vector<double>> path;
    for (int i=0;i<path.size();i++){
        geometry_msgs::Point p;
        p.x=path[i][0];
        p.y=path[i][1];
        p.z=0.0;
        Path.points.push_back(p);
    }
    pubmark.publish(Path);

}


Node RRT::generate_random_node(){
    // // 设置种子
    // std::srand(static_cast<unsigned int>(std::time(0)));
    // // 生成0到RAND_MAX之间的随机整数
    // int randomInteger=std::rand();
    // // 将随机整数映射到0到1之间
    // double randomZeroToOne=static_cast<double>(randomInteger)/RAND_MAX;
    // ROS_INFO("random=====%.3f",randomZeroToOne);
    std::random_device see;
    std::mt19937 pro(see());
    std::uniform_real_distribution<> ran_range(0,1000);
    double randomZeroToOne= (ran_range(pro))/1000;
    //ROS_INFO("random=====%.3f",randomZeroToOne);
    if (randomZeroToOne > goal_sample_rate){
         std::random_device seed;
         std::mt19937 prob(seed());
         std::uniform_real_distribution<> dos_xrange(x_range[0]+delta,x_range[1]-delta);
         std::uniform_real_distribution<> dos_yrange(y_range[0]+delta,y_range[1]-delta);
         double random_x= dos_xrange(prob);
         double random_y= dos_yrange(prob);
         //ROS_INFO("random_x==%.2f",random_x);
         Node random_node={random_x,random_y};
         return random_node;
    }
    return goal;
    
}

Node RRT::choose_parent(Node &node_new, vector<int> &neighbor_index){
    int cost_min_index;
    Node node_nearest;
    for (int i=0;i<neighbor_index.size();i++){
        int dex=neighbor_index[i];
        double cost=get_new_cost(vertex[dex],node_new);
        if (cost < node_new.cost){
            node_new.cost=cost;
            //cost_min_index=dex;
            //node_new.parent=vertex[dex].id;
            node_new.parent=vertex[dex].id;
            Node_gather[node_new.id]=node_new;
            node_nearest=vertex[dex];
            //ROS_INFO("给新节点更换父节点!!!!!!!!!!");
        }
    }
    //node_new.parent=vertex[cost_min_index].id;
    //node_new.cost=G;
    return node_nearest;
    
}

int RRT::search_goal_parent(){
    int node_index=vertex.size()-1;
    double G=1000000000;
    for (int i=0;i<vertex.size();i++){
        double dist_list=sqrt(pow(vertex[i].x-goal.x,2)+pow(vertex[i].y-goal.y,2));
        if (dist_list <= step_len && is_collision(vertex[i],goal)==false && 
                        (dist_list+vertex[i].cost)<G){
            G=dist_list+vertex[i].cost;
            node_index=i;
        }
    }
    return node_index;

}

void RRT::rewise(Node &node_new, vector<int> &neighbor_index ,int &id){
    for (int k=0; k<neighbor_index.size(); k++){
        int dex=neighbor_index[k];
        Node node_neighbor=vertex[dex];
        if (node_neighbor.cost > get_new_cost(node_new,node_neighbor)){
            //vertex[dex].parent=node_new.id;
            vertex[dex].cost=get_new_cost(node_new,node_neighbor);
            vertex[dex].parent=node_new.id;
            Node_gather[vertex[dex].id]=vertex[dex];
            //ROS_INFO("vertex[dex].parent==%.2f",vertex[dex].parent);
            revisualization_vertex(node_new,vertex[dex] , id);
            //ROS_INFO("重新布线!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
    }
}

double RRT::get_new_cost(Node &node_start, Node &node_end){
    vector<double> distance_and_angle=get_distance_and_angle(node_start,node_end);
    double dist=distance_and_angle[0];
    double cost=node_start.cost+dist;
    return cost;
}

vector<int> RRT::find_near_neighbor(Node &node_new){
    int n=vertex.size()+1;
    //double r=min(search_radius*sqrt(log(n)/n),step_len);
    double r=step_len*2;
    vector<int> dist_table_index;
    for (int i=0;i<vertex.size();i++){
        double dist=sqrt(pow(vertex[i].x-node_new.x,2)+pow(vertex[i].y-node_new.y,2));
        if (dist<=r && is_collision(node_new,vertex[i])==false){
            dist_table_index.push_back(i);
        }
    }
    return dist_table_index;
}

void RRT::planning(){
    int id=-15000;
    for (int uu=0;uu<iter_max;uu++){
        if ((uu+1) % 500==0){
            ROS_INFO("迭代次数:%d",uu+1);
        }
        
        Node node_rand=generate_random_node();
        //ROS_INFO("node_rand===%.2f",node_rand.x);
        Node node_near=nearest_neighbor(node_rand);
        //ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        Node node_new=new_state(node_near,node_rand,id);
        Node_gather[node_new.id]=node_new;
        id++;
        if (is_collision(node_near,node_new)==false){
            vector<int> neighbor_index=find_near_neighbor(node_new);
            visualization_vertex(node_near, node_new, id);
            
            if (neighbor_index.size()>0){
                Node node_nearest=choose_parent(node_new,neighbor_index);
                vertex.push_back(node_new);
                //visualization_vertex(node_nearest, node_new, id);
                rewise(node_new,neighbor_index,id);
            }
            // vector<double> distance_and_angle=get_distance_and_angle(node_new,goal);
            // double dist=distance_and_angle[0];
            // if (dist <= step_len && is_collision(node_new,goal)==false){
            //     extract_path(node_new);
            //     //ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
            //     visualization_path();
            //     break;
            // }
        }
    }
    int index=search_goal_parent();
    extract_path(vertex[index]);
    visualization_path();

}