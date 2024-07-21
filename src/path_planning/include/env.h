#include<iostream>
using namespace std;

class Point{
public:
    Point(int p1,int p2,double p3,double p4){parent_id=p1;id=p2;g_cost=p3;f_cost=p4;};
private:
    int parent_id;
    double g_cost;
    double f_cost;
    int id;
};