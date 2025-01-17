/**
 * @Author: YunKai Xia
 * @Date:   2022-07-10 22:28:51
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-07-10 22:37:25
 */

// Note: Thanks to TAI Lei for this data structure，Mail: ltai@ust.hk



#include <array>
#include <iostream>
#include <string>
#include <vector>
// #include "common.h"
#include "common.h"

#ifndef FRENET_PATH_H
#define FRENET_PATH_H

using Vec_f = std::vector<float>;
using Poi_f = std::array<float, 2>;
using Vec_Poi = std::vector<Poi_f>;

struct FrenetInitialConditions {
  double s0;
  double c_speed;
  double c_d;
  double c_d_d;
  double c_d_dd;
  double target_speed;
  //   double *wx;
  //   double *wy;
  //   int nw;
  //   double *o_llx;
  //   double *o_lly;
  //   double *o_urx;
  //   double *o_ury;
  //   int no;
};

class FrenetPath {
 public:
  float cd = 0.0;
  float cv = 0.0;
  float cf = 0.0;

  Vec_f t;      // time
  Vec_f d;      // lateral offset
  Vec_f d_d;    // lateral speed
  Vec_f d_dd;   // lateral acceleration
  Vec_f d_ddd;  // lateral jerk
  Vec_f s;      // s position along spline
  Vec_f s_d;    // s speed
  Vec_f s_dd;   // s acceleration
  Vec_f s_ddd;  // s jerk

  Vec_f x;    // x position
  Vec_f y;    // y position
  Vec_f yaw;  // yaw in rad
  Vec_f ds;   // speed
  Vec_f c;    // curvature

  float max_speed;
  float max_accel;
  float max_curvature;
};

using Vec_Path = std::vector<FrenetPath>;


#endif
