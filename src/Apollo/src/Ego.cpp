#include "Ego.h"
using namespace std;
void Ego_State_Update(Ego_State& ego , double dt, double steer){
    double vx=ego.vx*cos(ego.heading_xy) - ego.vy*sin(ego.heading_xy);
    double vy=ego.vx*sin(ego.heading_xy) + ego.vy*cos(ego.heading_xy);
    double ax=ego.ax*cos(ego.heading_xy) - ego.ay*sin(ego.heading_xy);
    double ay=ego.ax*sin(ego.heading_xy) + ego.ay*cos(ego.heading_xy);
    ego.x += vx*dt + 0.5*ax*dt*dt;
    ego.y += vy*dt + 0.5*ay*dt*dt;
    ego.vx += ax*dt;
    ego.vy += ay*dt;
    double v=sqrt(pow(vx,2) + pow(vy,2));
    ego.heading_xy += v*tan(steer)*dt/L;

}