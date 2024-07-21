#include "astar.h"
#include <map>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
// 为 std::pair<int, int> 提供哈希函数的特化
namespace std {
    template <>
    struct hash<std::pair<int, int>> {
        size_t operator()(const std::pair<int, int>& p) const {
            // 使用标准库提供的哈希函数将两个整数组合成一个哈希值
            return hash<int>()(p.first) ^ hash<int>()(p.second);
        }
    };
}


int prepathid=88888;
class DStar{
public:
    void dopoint(const geometry_msgs::PoseStamped::ConstPtr& msg){
        double x=std::round(msg->pose.position.x);
        double y=std::round(msg->pose.position.y);
        path.clear();
        if (x<0 || x>this->x-1 || y<0 || y>this->y-1){
            ROS_ERROR("Please choose right area!");
        }else{
            auto it=obs.find(Obs(x,y));
            if (it == obs.end()){
                ROS_INFO("Add obstacle at: s =%.2f , y =%.2f",x, y);

                this->obs.insert(Obs(x,y));
                update_obs(x, y);
                this->h[std::make_pair((int)x,(int)y)]=std::numeric_limits<double>::max();
                Node s=this->s_start;
                while (s.x!=this->s_goal.x || s.y!=this->s_goal.y){
                    if (is_collision(s,Node(PARENT[std::make_pair((int)s.x,(int)s.y)].first , PARENT[std::make_pair((int)s.x,(int)s.y)].second))){
                        //ROS_INFO("modify(s);");
                        modify(s);
                        continue;
                    }
                    s= Node(PARENT[std::make_pair((int)s.x,(int)s.y)].first , PARENT[std::make_pair((int)s.x,(int)s.y)].second);
                }
                extract_path(this->s_start,this->s_goal);
                grapath();
                
            }
        }
    }
    DStar(){
        this->env=std::shared_ptr<Env>(new Env());
        this->u_set=this->env->getmotios();
        this->obs=this->env->getobs();
        this->x=this->env->getxrange();
        this->y=this->env->getyrange();

        s_start=Node(5,5);
        s_goal=Node(45,25);


        pub_mark=nh_.advertise<visualization_msgs::Marker>("visualization_marker",100);
        pub_markarray=nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",100);
        sub_point=nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,[this](const geometry_msgs::PoseStamped::ConstPtr& msg){dopoint(msg);});
        
        dstarpath.header.frame_id="map";
        dstarpath.header.stamp=ros::Time::now();
        dstarpath.type=visualization_msgs::Marker::LINE_STRIP;
        dstarpath.action=visualization_msgs::Marker::ADD;
        dstarpath.id=0;
        dstarpath.scale.x=0.2;
        dstarpath.color.r=0.0;
        dstarpath.color.g=1.0;
        dstarpath.color.b=0.0;
        dstarpath.color.a=1.0;
        
    }
    ~DStar()=default;
    void init(){
        for (int i=0;i<this->env->getxrange();i++){
            for (int j=0;j<this->env->getyrange();j++){
                this->t[std::make_pair(i,j)]="NEW";
                this->k[std::make_pair(i,j)]=0.0;
                this->h[std::make_pair(i,j)]=std::numeric_limits<double>::max();
                this->PARENT[std::make_pair(i,j)]={-1,-1};
            }
        }
        h[std::make_pair((int)s_goal.x,(int)s_goal.y)]=0.0;
    }
    void insert(Node s, double h_new){
        if (this->t[std::make_pair((int)s.x,(int)s.y)]=="NEW"){
            this->k[std::make_pair((int)s.x,(int)s.y)]=h_new;
        }else if (this->t[std::make_pair((int)s.x,(int)s.y)]=="OPEN"){
            this->k[std::make_pair((int)s.x,(int)s.y)]=
                                std::min(this->k[std::make_pair((int)s.x,(int)s.y)],h_new);
        }else if (this->t[std::make_pair((int)s.x,(int)s.y)]=="CLOSED"){
            this->k[std::make_pair((int)s.x,(int)s.y)]=
                                std::min(h[std::make_pair((int)s.x,(int)s.y)],h_new);
        }
        this->h[std::make_pair((int)s.x,(int)s.y)]=h_new;
        this->t[std::make_pair((int)s.x,(int)s.y)]="OPEN";
        this->OPEN.push_back(s);
    }
    void Delete(Node s){
        if (this->t[std::make_pair((int)s.x,(int)s.y)]=="OPEN"){
            this->t[std::make_pair((int)s.x,(int)s.y)]="CLOSED";
        }
        auto it=this->OPEN.begin();
        for (; it!=this->OPEN.end(); it++){
            if (it->x==s.x && it->y==s.y){
                this->OPEN.erase(it);
                break;
            }
        }
    }
    void modify_cost(Node s){
        if (this->t[std::make_pair((int)s.x,(int)s.y)]=="CLOSED"){
            this->insert(s, h[PARENT[std::make_pair((int)s.x,(int)s.y)]] +
                             cost(s,PARENT[std::make_pair((int)s.x,(int)s.y)]));
        }
        
    }
    Node min_state(){
        Node s;
        if (OPEN.empty()) return s;
        double maxcost=std::numeric_limits<double>::max();
        for (Node node:OPEN){
            if (k[std::make_pair((int)node.x,(int)node.y)] < maxcost){
                maxcost=k[std::make_pair((int)node.x,(int)node.y)];
                s=node;
            }
        }
        return s;
    }
    double get_k_min(){
        if (OPEN.empty()) return -1;
        double maxcost=std::numeric_limits<double>::max();
        for (Node node:OPEN){
            if (k[std::make_pair((int)node.x,(int)node.y)] < maxcost){
                maxcost=k[std::make_pair((int)node.x,(int)node.y)];
            }
        }
        return maxcost;
    }
    double process_state(){
        Node s=this->min_state();
        //std::cout<<"s.x="<<s.x<<"  s.y="<<s.y<<std::endl;
        if (s.id==-1) return -1;
        double k_old = this->get_k_min();
        this->Delete(s);
        if (k_old < h[std::make_pair((int)s.x,(int)s.y)]){
            for (Node s_n:this->get_neighbor(s)){
                if (h[std::make_pair((int)s_n.x,(int)s_n.y)]<=k_old && 
                                h[std::make_pair((int)s.x,(int)s.y)]>h[std::make_pair((int)s_n.x,(int)s_n.y)]+cost(s_n,s)){
                    PARENT[std::make_pair((int)s.x,(int)s.y)]=std::make_pair((int)s_n.x,(int)s_n.y);
                    h[std::make_pair((int)s.x,(int)s.y)]=h[std::make_pair((int)s_n.x,(int)s_n.y)]+cost(s_n,s);
                }
            }
        }
        if (k_old==h[std::make_pair((int)s.x,(int)s.y)]){
            for (Node s_n:this->get_neighbor(s)){
                if (t[std::make_pair((int)s_n.x,(int)s_n.y)]=="NEW" || (PARENT[std::make_pair((int)s_n.x,(int)s_n.y)]==std::make_pair((int)s.x,(int)s.y) && 
                                h[std::make_pair((int)s_n.x,(int)s_n.y)]!=h[std::make_pair((int)s.x,(int)s.y)] + cost(s, s_n)) ||
                                (PARENT[std::make_pair((int)s_n.x,(int)s_n.y)]!=std::make_pair((int)s.x,(int)s.y) && 
                                h[std::make_pair((int)s_n.x,(int)s_n.y)]>h[std::make_pair((int)s.x,(int)s.y)] + cost(s, s_n))){
                    
                    PARENT[std::make_pair((int)s_n.x,(int)s_n.y)]=std::make_pair((int)s.x,(int)s.y);
                    insert(s_n, h[std::make_pair((int)s.x,(int)s.y)] + cost(s, s_n));
                }
            }
        }else{
            for (Node s_n:this->get_neighbor(s)){
                if (t[std::make_pair((int)s_n.x,(int)s_n.y)]=="NEW" || 
                            (PARENT[std::make_pair((int)s_n.x,(int)s_n.y)]==std::make_pair((int)s.x,(int)s.y) &&
                            h[std::make_pair((int)s_n.x,(int)s_n.y)]!=h[std::make_pair((int)s.x,(int)s.y)] + cost(s, s_n))){
                    
                    PARENT[std::make_pair((int)s_n.x,(int)s_n.y)] = std::make_pair((int)s.x,(int)s.y);
                    insert(s_n, h[std::make_pair((int)s.x,(int)s.y)] + cost(s, s_n));
                }else{
                    if (PARENT[std::make_pair((int)s_n.x,(int)s_n.y)]!=std::make_pair((int)s.x,(int)s.y) &&
                                h[std::make_pair((int)s_n.x,(int)s_n.y)] > h[std::make_pair((int)s.x,(int)s.y)] + cost(s, s_n)){
                        insert(s, h[std::make_pair((int)s.x,(int)s.y)]);
                    }else{
                        if (PARENT[std::make_pair((int)s_n.x,(int)s_n.y)]!=std::make_pair((int)s.x,(int)s.y) &&
                            h[std::make_pair((int)s.x,(int)s.y)] > h[std::make_pair((int)s_n.x,(int)s_n.y)]+cost(s_n, s) &&
                            t[std::make_pair((int)s_n.x,(int)s_n.y)] == "CLOSED" && h[std::make_pair((int)s_n.x,(int)s_n.y)] > k_old){
                            
                            insert(s_n, h[std::make_pair((int)s_n.x,(int)s_n.y)]);
                        }
                    }
                }
            }
        }
        return get_k_min();
    }
    void modify(Node s){
        this->modify_cost(s);
        while (true){
            double k_min=this->process_state();
            if (k_min>=h[std::make_pair((int)s.x,(int)s.y)]) break;
        }
    }
    std::vector<Node> get_neighbor(Node s){
       std::vector<Node>  nei_list;
       for (auto u:u_set){
            Node s_next(s.x+u[0] , s.y+u[1]);
            nei_list.push_back(s_next);
       }
       return nei_list;
    }
    double cost(Node s_start, Node s_goal){
        if (is_collision(s_start, s_goal)){
            return std::numeric_limits<double>::max();
        }
        return std::hypot(s_goal.x-s_start.x , s_goal.y-s_start.y);
    }
    double cost(Node s_start, std::pair<int,int> goal){
        Node s_goal(goal.first,goal.second);
        if (is_collision(s_start, s_goal)){
            return std::numeric_limits<double>::max();
        }
        return std::hypot(s_goal.x-s_start.x , s_goal.y-s_start.y);
    }
    bool is_collision(const Node& s_start, const Node& s_end){
        auto it_start=obs.find(Obs(s_start.x,s_start.y));
        auto it_end=obs.find(Obs(s_end.x, s_end.y));
        if (it_start != obs.end() || it_end != obs.end()) return true;
        if (s_start.x != s_end.x && s_start.y != s_end.y){
            Node s1, s2;
            if ((s_end.x-s_start.x) == (s_start.y-s_end.y)){
                s1.x=std::min(s_start.x , s_end.x);
                s1.y=std::min(s_start.y , s_end.y);
                s2.x=std::max(s_start.x , s_end.x);
                s2.y=std::max(s_start.y , s_end.y);
            }else{
                s1.x=std::min(s_start.x , s_end.x);
                s1.y=std::max(s_start.y , s_end.y);
                s2.x=std::max(s_start.x , s_end.x);
                s2.y=std::min(s_start.y , s_end.y);
            }
            auto it_s1=obs.find(Obs(s1.x,s1.y));
            auto it_s2=obs.find(Obs(s2.x,s2.y));
            if (it_s1 != obs.end() || it_s2 != obs.end()) return true;
        }
        return false;
    }
    void extract_path(Node s_start, Node s_end){
        path.push_back(s_start);
        std::pair<int,int> s_=std::make_pair((int)s_start.x,(int)s_start.y);
        while (ros::ok()){
            s_=PARENT[s_];
            auto it=obs.find(Obs(s_.first,s_.second));
            if (it!=obs.end()){
                ROS_ERROR("!!!!!!!!!!!!!!!!!");
            }
            path.push_back(Node(s_.first,s_.second));
            //std::cout<<"pathx="<<s_.first<<"  pathy="<<s_.second<<std::endl;
            if (s_.first==(int)s_end.x && s_.second==(int)s_end.y) break;
        }
        //std::cout<<"path.size="<<path.size()<<std::endl;
    }
    void run(Node s_start, Node s_end){
        this->init();
        insert(s_end, 0);
        while (ros::ok()){
            process_state();
            if (t[std::make_pair((int)s_start.x,(int)s_start.y)]=="CLOSED") break;
        }
        extract_path(s_start, s_end);
        grapath();
    }
    std::vector<Node> getpath(){return this->path;};
    void graphobs(){
        for (Obs ob:obs){
            visualization_msgs::Marker obsi;
            obsi.header.frame_id="map";
            obsi.header.stamp=ros::Time::now();
            obsi.id=index;
            obsi.type=visualization_msgs::Marker::CUBE;
            obsi.action=visualization_msgs::Marker::ADD;
            obsi.scale.x=1.0;
            obsi.scale.y=1.0;
            obsi.scale.z=0.0;
            obsi.color.r=1.0;
            obsi.color.g=0.0;
            obsi.color.b=0.0;
            obsi.color.a=1.0;
            obsi.pose.position.x=ob.x;
            obsi.pose.position.y=ob.y;
            obsi.pose.position.z=0.0;
            obsi.pose.orientation.w=1.0;
            obsi.pose.orientation.x=0.0;
            obsi.pose.orientation.y=0.0;
            obsi.pose.orientation.z=0.0;
            obstacle.markers.push_back(obsi);
            index++;
        }
        for (int i=0;i<3;i++){
            pub_markarray.publish(obstacle);
            ros::Rate(5).sleep();
        }
        
    }
    
    void update_obs(double x, double y){
        visualization_msgs::Marker obsi;
        obsi.header.frame_id="map";
        obsi.header.stamp=ros::Time::now();
        obsi.id=index;
        obsi.type=visualization_msgs::Marker::CUBE;
        obsi.action=visualization_msgs::Marker::ADD;
        obsi.scale.x=1.0;
        obsi.scale.y=1.0;
        obsi.scale.z=0.0;
        obsi.color.r=1.0;
        obsi.color.g=0.0;
        obsi.color.b=0.0;
        obsi.color.a=1.0;
        obsi.pose.position.x=x;
        obsi.pose.position.y=y;
        obsi.pose.position.z=0.0;
        obsi.pose.orientation.w=1.0;
        obsi.pose.orientation.x=0.0;
        obsi.pose.orientation.y=0.0;
        obsi.pose.orientation.z=0.0;
        obstacle.markers.push_back(obsi);
        index++;
        for (int i=0;i<3;i++){
            pub_markarray.publish(obstacle);
            ros::Rate(5).sleep();
        }
    }

    void grapath(){
        dstarpath.points.clear();
        dstarpath.header.stamp = ros::Time::now();
        dstarpath.header.frame_id = "map"; // 设置帧
        geometry_msgs::Point p;
        for (int i=0;i<path.size();i++){
            Node node=path[i];
            p.x=node.x;
            p.y=node.y;
            p.z=0.2;
            dstarpath.points.push_back(p);
        } 
        
        pub_mark.publish(dstarpath);
    }

    
private:
    Node s_start, s_goal;
    std::shared_ptr<Env> env;
    std::vector<std::vector<double>> u_set;
    std::unordered_set<Obs> obs;
    double x;
    double y;
    std::vector<Node> OPEN;
    std::unordered_map<std::pair<int,int>,std::pair<int,int>> PARENT;
    std::map<std::pair<int,int>,std::string> t;
    std::map<std::pair<int,int>,double> h;
    std::map<std::pair<int,int>,double> k;
    std::vector<Node> path;
    int count=0;
    int index=1;
    ros::NodeHandle nh_;
    ros::Publisher pub_mark;
    ros::Publisher pub_markarray;
    ros::Subscriber sub_point;
    visualization_msgs::Marker dstarpath;
    visualization_msgs::MarkerArray obstacle;
};