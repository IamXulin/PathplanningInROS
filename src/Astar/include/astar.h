#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <functional>
#include <thread>
#include <chrono>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
struct Node
{
    double x;
    double y;
    Node()=default;
    Node (double x, double y){
        this->x=x;
        this->y=y;
        this->id=(int)x+(int)y*51;
    }
    double cost;
    int id=-1;
    bool operator<(const Node& other) const {
        return cost>other.cost;
    }
    bool operator==(const Node& other) const {
        return (other.x==this->x && other.y==this->y);
    }
};
namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& n) const {
            // 将 Node 对象的成员变量转换为哈希值
            size_t hash_x = hash<double>()(n.x);
            size_t hash_y = hash<double>()(n.y);
            size_t hash_cost = hash<double>()(n.cost);
            size_t hash_id = hash<int>()(n.id);
            // 使用混合哈希函数
            size_t seed = hash_x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_y + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_cost + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash_id + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
}

struct Obs {
    double x;
    double y;

    // 默认构造函数
    Obs() = default;

    Obs(double x, double y) : x(x), y(y) {}

    bool operator==(const Obs& other) const {
        return (x == other.x && y == other.y);
    }
};
// 哈希函数特化
namespace std {
    template <>
    struct hash<Obs> {
        size_t operator()(const Obs& obs) const {
            size_t h1 = std::hash<double>{}(obs.x);
            size_t h2 = std::hash<double>{}(obs.y);
            return h1 ^ (h2 << 1);
        }
    };
}


class Env{
public:
    Env() : x_range(51), y_range(31) {
        motions = {{-1, 0}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}};
        obs_map();
    }
    double getxrange(){return this->x_range;};
    double getyrange(){return this->y_range;};
    ~Env() = default;
    void obs_map(){
        double x=this->x_range;
        double y=this->y_range;
        for (int i=0;i<x;i++){
            this->obs.insert(Obs(i,0));
        }
        for (int i=0;i<x;i++){
            this->obs.insert(Obs(i,y-1));
        }
        for (int i=0;i<y;i++){
            this->obs.insert(Obs(0,i));
        }
        for (int i=0;i<y;i++){
            this->obs.insert(Obs(x-1,i));
        }
        for (int i=10;i<21;i++){
            this->obs.insert(Obs(i,15));
        }
        for (int i=0;i<15;i++){
            this->obs.insert(Obs(20,i));
        }
        for (int i=15;i<30;i++){
            this->obs.insert(Obs(30,i));
        }
        for (int i=0;i<16;i++){
            this->obs.insert(Obs(40,i));
        }
        return;
    }
    void update_obs(std::unordered_set<Obs> obs){
        this->obs=obs;
    }
    std::unordered_set<Obs> getobs() {return this->obs;};
    std::vector<std::vector<double>> getmotios() {return this->motions;};
private:
    double x_range;
    double y_range;
    std::vector<std::vector<double>> motions;
    std::unordered_set<Obs> obs;
};


class AStar{
public:
    AStar(std::string heuristic_type){
        // this->s_start=s_start;
        // this->s_goal=s_goal;
        this->heuristic_type=heuristic_type;
        this->env=std::shared_ptr<Env>(new Env());
        this->u_set=env->getmotios();
        this->obs=env->getobs();


        pub_mark=nh_.advertise<visualization_msgs::Marker>("visualization_marker",100);
        pub_markarray=nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",100);
        astarpath.header.frame_id="map";
        astarpath.header.stamp=ros::Time::now();
        astarpath.type=visualization_msgs::Marker::LINE_STRIP;
        astarpath.action=visualization_msgs::Marker::ADD;
        astarpath.id=0;
        astarpath.scale.x=0.2;
        astarpath.color.r=0.0;
        astarpath.color.g=1.0;
        astarpath.color.b=0.0;
        astarpath.color.a=1.0;

        updateobs();
        
    }
    double heuristic(const Node& s){
        if (this->heuristic_type=="manhattan"){
            return std::abs(s_goal.x-s.x) + std::abs(s_goal.y-s.y);
        }else{
            return std::hypot(s_goal.x-s.x , s_goal.y-s.y);
        }
    }
    std::vector<std::vector<double>> get_neighbor(const Node& s){
        std::vector<std::vector<double>> neighbor;
        for (const auto& u:this->u_set){
            neighbor.push_back({s.x+u[0] , s.y+u[1]});
        }
        return neighbor;
    }
    double cost(const Node& s_start, const Node& s_goal){
        if (this->is_collision(s_start , s_goal)) return std::numeric_limits<double>::max();
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
    double f_value(const Node& s){
        return g[s.id] + heuristic(s);
    }
    std::vector<Node> extract_path(){
        this->path.clear();
        Node s=this->s_goal;
        this->path.push_back(this->s_goal);
        
        while (true){
            std::cout<<"curx="<<s.x<<"  cury="<<s.y<<std::endl;
            int preid=PARENT[s.id];
            for (const auto& node:CLOSE){
                if (node.id==preid){
                    s=node;
                }
            }
            this->path.push_back(s);
            if (s.x==this->s_start.x && s.y==this->s_start.y) break;
            //std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        return this->path;
    }
    std::vector<Node> searching(){
        PARENT[this->s_start.id]=s_start.x+s_start.y*51;
        this->g[this->s_start.id]=0;
        this->g[this->s_goal.id]=std::numeric_limits<double>::max();
        this->s_start.cost=this->f_value(this->s_start);
        this->OPEN.push(this->s_start);
        
        while (!this->OPEN.empty()){
            Node s=this->OPEN.top();
            this->OPEN.pop();
            this->CLOSE.push_back(s);
            std::cout<<"curx="<<s.x<<"  cury="<<s.y<<std::endl;
            if (s.x==this->s_goal.x && s.y==this->s_goal.y) break; 
            //std::this_thread::sleep_for(std::chrono::milliseconds(2));
            for (const auto& s_:this->get_neighbor(s)){
                
                Node s_n(s_[0],s_[1]);
                s_n.id=s_n.x+s_n.y*51;
                double new_cost=this->g[s.id] + this->cost(s,s_n);
                
                auto it_sn=this->g.find(s_n.id);
                
                if (it_sn == this->g.end()) this->g[s_n.id]=std::numeric_limits<double>::max();
                if (new_cost < this->g[s_n.id]){
                    this->g[s_n.id]=new_cost;
                    this->PARENT[s_n.id]=s.id;
                    s_n.cost=this->f_value(s_n);
                    this->OPEN.push(s_n);
                    updateopen(s_n);
                }
            }
            

        }
        return this->extract_path();
    }
    void updateobs(){
        int i=10000;
        for (const auto& o:this->obs){
            visualization_msgs::Marker obs_;
            obs_.header.frame_id="map";
            obs_.header.stamp=ros::Time::now();
            obs_.type=visualization_msgs::Marker::CUBE;
            obs_.action=visualization_msgs::Marker::ADD;
            obs_.id=i;
            obs_.scale.x=1.0;
            obs_.scale.y=1.0;
            obs_.scale.z=0.0;
            obs_.color.r=1.0;
            obs_.color.g=0.0;
            obs_.color.b=0.0;
            obs_.color.a=1.0;
            obs_.pose.position.x=o.x;
            obs_.pose.position.y=o.y;
            obs_.pose.position.z=0.0;
            obs_.pose.orientation.w=1.0;
            obs_.pose.orientation.x=0.0;
            obs_.pose.orientation.y=0.0;
            obs_.pose.orientation.z=0.0;
            i--;
            open.markers.push_back(obs_);
        }
        for (int i=0;i<3;i++){
            pub_markarray.publish(open);
            ros::Rate(5).sleep();
        }
        
    }
    void updateopen(const Node& s){
        visualization_msgs::Marker s_;
        s_.header.frame_id="map";
        s_.header.stamp=ros::Time::now();
        s_.type=visualization_msgs::Marker::CUBE;
        s_.action=visualization_msgs::Marker::ADD;
        s_.id=s.id;
        s_.scale.x=0.5;
        s_.scale.y=0.5;
        s_.scale.z=0.0;
        s_.color.r=0.3;
        s_.color.g=0.5;
        s_.color.b=0.8;
        s_.color.a=1.0;
        s_.pose.position.x=s.x;
        s_.pose.position.y=s.y;
        s_.pose.position.z=0.0;
        s_.pose.orientation.w=1.0;
        s_.pose.orientation.x=0.0;
        s_.pose.orientation.y=0.0;
        s_.pose.orientation.z=0.0;
        open.markers.push_back(s_);
        pub_markarray.publish(open);
    }
    void grapath(){
        geometry_msgs::Point p;
        for (const auto& line:this->path){
            p.x=line.x;
            p.y=line.y;
            p.z=0.2;
            astarpath.points.push_back(p);
        }

        pub_mark.publish(astarpath);
        ros::Duration(0.1).sleep();
    }
    void AddNode(Node start, Node goal){
        this->s_start=start;
        this->s_goal=goal;
    }
private:
    Node s_start;
    Node s_goal;
    ros::NodeHandle nh_;
    ros::Publisher pub_mark;
    ros::Publisher pub_markarray;
    visualization_msgs::Marker astarpath;
    visualization_msgs::MarkerArray open;
    std::string heuristic_type;
    std::shared_ptr<Env> env;
    std::vector<std::vector<double>> u_set;
    std::unordered_set<Obs> obs;
    std::unordered_map<int,double> g;
    std::priority_queue<Node> OPEN;
    std::vector<Node> CLOSE;
    std::unordered_map<int,int> PARENT;
    std::vector<Node> path;
};
