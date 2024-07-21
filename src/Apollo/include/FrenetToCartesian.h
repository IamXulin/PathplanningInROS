#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <random>
#include <limits>
#include <cmath>
#include <tuple>
using namespace std;

#ifndef FRETOCAR_H
#define FRETOCAR_H

std::tuple<double,double,double,double> CalcProjPoint(double s, 
        const vector<double> &frenet_path_x, const vector<double> &frenet_path_y,
        const vector<double> &frenet_path_heading, const vector<double> &frenet_path_kappa,
        const vector<double> &index2s);

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>>
        FrenetToCartesian(const vector<double> &s_set, const vector<double> &l_set,
        const vector<double> &dl_set, const vector<double> &ddl_set,
        const vector<double> &frenet_path_x, const vector<double> &frenet_path_y,
        const vector<double> &frenet_path_heading, const vector<double> &frenet_path_kappa,
        const vector<double> &index2s);



#endif