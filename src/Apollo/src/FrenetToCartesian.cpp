#include "FrenetToCartesian.h"

std::tuple<double,double,double,double> CalcProjPoint(double s, 
        const vector<double> &frenet_path_x, const vector<double> &frenet_path_y,
        const vector<double> &frenet_path_heading, const vector<double> &frenet_path_kappa,
        const vector<double> &index2s){
    int match_index=0;
    while (index2s[match_index] < s){
        match_index+=1;
    }
    double match_point[2]={frenet_path_x[match_index] , frenet_path_y[match_index]};
    double match_point_heading=frenet_path_heading[match_index];
    double match_point_kappa=frenet_path_kappa[match_index];
    double ds=s - index2s[match_index];
    double match_tor[2]={cos(match_point_heading) , sin(match_point_heading)};
    double proj_x=match_point[0] + ds*match_tor[0];
    double proj_y=match_point[1] + ds*match_tor[1];
    double proj_heading=match_point_heading + ds*match_point_kappa;
    double proj_kappa=match_point_kappa;
    return std::make_tuple(proj_x, proj_y, proj_heading, proj_kappa);
}

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>>
        FrenetToCartesian(const vector<double> &s_set, const vector<double> &l_set,
        const vector<double> &dl_set, const vector<double> &ddl_set,
        const vector<double> &frenet_path_x, const vector<double> &frenet_path_y,
        const vector<double> &frenet_path_heading, const vector<double> &frenet_path_kappa,
        const vector<double> &index2s){
    vector<double> x_set(600,std::nan(""));
    vector<double> y_set(600,std::nan(""));
    vector<double> heading_set(600,std::nan(""));
    vector<double> kappa_set(600,std::nan(""));

    for (int i=0;i<s_set.size();i++){
        if (std::isnan(s_set[i])) break;
        auto proj_point=CalcProjPoint(s_set[i], frenet_path_x, frenet_path_y,
                                        frenet_path_heading, frenet_path_kappa, index2s);
        
        double proj_x=std::get<0>(proj_point);
        double proj_y=std::get<1>(proj_point);
        double proj_heading=std::get<2>(proj_point);
        double proj_kappa=std::get<3>(proj_point);
        
        double nor[2]={-sin(proj_heading) , cos(proj_heading)};
        x_set[i]=proj_x + l_set[i]*nor[0];
        y_set[i]=proj_y + l_set[i]*nor[1];
        heading_set[i]=proj_heading + atan(dl_set[i]/(1-proj_kappa*l_set[i]));
        kappa_set[i]=((ddl_set[i] + proj_kappa*dl_set[i]*tan(heading_set[i] - proj_heading))*
                    pow(cos(heading_set[i] - proj_heading),2)/(1-proj_kappa*l_set[i]) + proj_kappa)*
                    cos(heading_set[i] - proj_heading)/(1-proj_kappa*l_set[i]);

    }
    return std::make_tuple(x_set, y_set, heading_set, kappa_set);
}

