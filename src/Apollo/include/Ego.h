#include <iostream>

#ifndef EGO_H
#define EGO_H
#define L 3.0
struct Ego_State
{
    double x;
    double y;
    double heading_xy;
    double vx;
    double vy;
    double ax;
    double ay;
};

void Ego_State_Update(Ego_State& ego , double dt, double steer);



#endif