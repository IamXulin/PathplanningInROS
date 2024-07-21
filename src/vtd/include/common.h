/**
 * @Author: YunKai Xia
 * @Date:   2022-07-10 22:28:51
 * @Last Modified by:   Runqi Qiu
 * @Last Modified time: 2022-10-29 22:33:03
 */
//#include <lgsvl_msgs/VehicleControlData.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <iterator>
#include <fstream>
#include <iostream>
#include <string>
#include <array>
#include <Eigen/Eigen>
#include "reference_line.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <string>

#ifndef COMMON_H
#define COMMON_H

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using std_msgs::ColorRGBA;
struct VehicleState {
  double x;
  double y;
  double heading;           // 车辆朝向
  double kappa;             // 曲率(切线斜率)
  double velocity;          // 速度
  double angular_velocity;  // 角速度
  double acceleration;      // 加速度

  // 规划起点
  double planning_init_x;
  double planning_init_y;

  double roll;
  double pitch;
  double yaw;

  double target_curv;  // 期望点的曲率

  double vx;
  double vy;

  // added
  double start_point_x;
  double start_point_y;

  double relative_x = 0;
  double relative_y = 0;

  double relative_distance = 0;
};

struct TrajectoryPoint {
  double x;
  double y;
  double heading;
  double kappa;
  double v;
  double a;
};

// 轨迹
struct TrajectoryData {
  std::vector<TrajectoryPoint> trajectory_points;
};

struct LateralControlError {
  double lateral_error;       // 横向误差
  double heading_error;       // 转向误差
  double lateral_error_rate;  // 横向误差速率
  double heading_error_rate;  // 转向误差速度
};

struct ControlCmd {
  double steer_target;
  double acc;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

struct tPoint {
  double t;
};
typedef std::vector<double> T_Points;

struct sPoint {
  double s;
  double s_d;
  double s_dd;
  double s_ddd;
  double cd = 0.0;
  double cv = 0.0;
  double cf = 0.0;
};
typedef std::vector<sPoint> S_Points;

struct dPoint {
  double d;
  double d_d;
  double d_dd;
  double d_ddd;
};
typedef std::vector<dPoint> D_Points;

struct xPoint {
  double x;
  double y;
  double yaw;
  double ds;
  double c;
};
typedef std::vector<xPoint> X_Points;

typedef std::shared_ptr<LateralControlError> LateralControlErrorPtr;

using Vec_f = std::vector<float>;
using Poi_f = std::array<float, 2>;
using Vec_Poi = std::vector<Poi_f>;



inline Vec_f vec_diff(Vec_f input) {
  Vec_f output;
  for (unsigned int i = 1; i < input.size(); i++) {
    output.push_back(input[i] - input[i - 1]);
  }
  return output;
};

inline Vec_f cum_sum(Vec_f input) {
  Vec_f output;
  float temp = 0;
  for (unsigned int i = 0; i < input.size(); i++) {
    temp += input[i];
    output.push_back(temp);
  }
  return output;
};

class Spline {
 public:
  Vec_f x;
  Vec_f y;
  int nx;
  Vec_f h;
  Vec_f a;
  Vec_f b;
  Vec_f c;
  // Eigen::VectorXf c;
  Vec_f d;

  Spline(){};
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(Vec_f x_, Vec_f y_)
      : x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_) {
    Eigen::MatrixXf A = calc_A();
    Eigen::VectorXf B = calc_B();
    Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
    float* c_pointer = c_eigen.data();
    // Eigen::Map<Eigen::VectorXf>(c, c_eigen.rows(), 1) = c_eigen;
    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (int i = 0; i < nx - 1; i++) {
      d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
      b.push_back((a[i + 1] - a[i]) / h[i] -
                  h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
    }
  };

  float calc(float t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    float dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx +
           d[seg_id] * dx * dx * dx;
  };

  float calc_d(float t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx - 1);
    float dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  float calc_dd(float t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    float dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

 private:
  Eigen::MatrixXf calc_A() {
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
    A(0, 0) = 1;
    for (int i = 0; i < nx - 1; i++) {
      if (i != nx - 2) {
        A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
      }
      A(i + 1, i) = h[i];
      A(i, i + 1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx - 1, nx - 2) = 0.0;
    A(nx - 1, nx - 1) = 1.0;
    return A;
  };
  Eigen::VectorXf calc_B() {
    Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);
    for (int i = 0; i < nx - 2; i++) {
      B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] -
                 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    return B;
  };

  int bisect(float t, int start, int end) {
    int mid = (start + end) / 2;
    if (t == x[mid] || end - start <= 1) {
      return mid;
    } else if (t > x[mid]) {
      return bisect(t, mid, end);
    } else {
      return bisect(t, start, mid);
    }
  }
};

class Spline2D {
 public:
  Spline sx;
  Spline sy;
  Vec_f s;

  Spline2D(Vec_f x, Vec_f y) {
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
  };

  Poi_f calc_postion(float s_t) {
    float x = sx.calc(s_t);
    float y = sy.calc(s_t);
    return {{x, y}};
  };

  float calc_curvature(float s_t) {
    float dx = sx.calc_d(s_t);
    float ddx = sx.calc_dd(s_t);
    float dy = sy.calc_d(s_t);
    float ddy = sy.calc_dd(s_t);
    return (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
  };

  float calc_yaw(float s_t) {
    float dx = sx.calc_d(s_t);
    float dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
  };

 private:
  Vec_f calc_s(Vec_f x, Vec_f y) {
    Vec_f ds;
    Vec_f out_s{0};
    Vec_f dx = vec_diff(x);
    Vec_f dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++) {
      ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    Vec_f cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  };
};

class QuarticPolynomial {
 public:
  // current parameter at t=0
  float xs;
  float vxs;
  float axs;

  // parameters at target t=t_j
  float vxe;
  float axe;

  // function parameters
  float a0, a1, a2, a3, a4;

  QuarticPolynomial(){};

  QuarticPolynomial(float xs_, float vxs_, float axs_, float vxe_, float axe_,
                    float T)
      : xs(xs_),
        vxs(vxs_),
        axs(axs_),
        vxe(vxe_),
        axe(axe_),
        a0(xs_),
        a1(vxs_),
        a2(axs_ / 2.0) {
    Eigen::Matrix2f A;
    A << 3 * std::pow(T, 2), 4 * std::pow(T, 3), 6 * T, 12 * std::pow(T, 2);
    Eigen::Vector2f B;
    B << vxe - a1 - 2 * a2 * T, axe - 2 * a2;

    Eigen::Vector2f c_eigen = A.colPivHouseholderQr().solve(B);
    a3 = c_eigen[0];
    a4 = c_eigen[1];
  };

  float calc_point(float t) {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4);
  };

  float calc_first_derivative(float t) {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3);
  };

  float calc_second_derivative(float t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2);
  };

  float calc_third_derivative(float t) { return 6 * a3 + 24 * a4 * t; };
};


class QuinticPolynomial {
 public:
  // current parameter at t=0
  float xs;
  float vxs;
  float axs;

  // parameters at target t=t_j
  float xe;
  float vxe;
  float axe;

  // function parameters
  float a0, a1, a2, a3, a4, a5;

  QuinticPolynomial(){};

  // polynomial parameters
  QuinticPolynomial(float xs_, float vxs_, float axs_, float xe_, float vxe_,
                    float axe_, float T)
      : xs(xs_),
        vxs(vxs_),
        axs(axs_),
        xe(xe_),
        vxe(vxe_),
        axe(axe_),
        a0(xs_),
        a1(vxs_),
        a2(axs_ / 2.0) {
    Eigen::Matrix3f A;
    A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 3 * std::pow(T, 2),
        4 * std::pow(T, 3), 5 * std::pow(T, 4), 6 * T, 12 * std::pow(T, 2),
        20 * std::pow(T, 3);
    Eigen::Vector3f B;
    B << xe - a0 - a1 * T - a2 * std::pow(T, 2), vxe - a1 - 2 * a2 * T,
        axe - 2 * a2;

    Eigen::Vector3f c_eigen = A.colPivHouseholderQr().solve(B);
    a3 = c_eigen[0];
    a4 = c_eigen[1];
    a5 = c_eigen[2];
  };

  float calc_point(float t) {
    return a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
  };

  float calc_first_derivative(float t) {
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) + 4 * a4 * std::pow(t, 3) +
           a5 * std::pow(t, 4);
  };

  float calc_second_derivative(float t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * std::pow(t, 2) +
           20 * a5 * std::pow(t, 3);
  };

  float calc_third_derivative(float t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * std::pow(t, 2);
  };
};

#endif