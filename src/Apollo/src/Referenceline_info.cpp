#include "Referenceline_info.h"

vector<double> Get_Index2s(const vector<double> &reference_x,const vector<double> &reference_y,
        const double &origin_x, const double &origin_y, const int &match_index){
    int n=181;
    vector<double> index2s(n,0.0);
    for (int i=1;i<n;i++){
        double dis=sqrt(pow(reference_x[i]-reference_x[i-1],2) +
                         pow(reference_y[i]-reference_y[i-1],2));
        dis+=index2s[i-1];
        index2s[i]=dis;
    }
    double s_temp=index2s[match_index];
    double vector_match_to_origin[2]={origin_x-reference_x[match_index],
                                      origin_y-reference_y[match_index]};
    double vector_match_to_match_next[2]={reference_x[match_index+1]-
                reference_x[match_index],reference_y[match_index+1]-
                reference_y[match_index]};
    double flag=vector_match_to_origin[0]*vector_match_to_match_next[0]+
                vector_match_to_origin[1]*vector_match_to_match_next[1];
    double s0;
    if (flag > 0){
        s0=s_temp + sqrt(vector_match_to_origin[0]*vector_match_to_origin[0]+
                         vector_match_to_origin[1]*vector_match_to_origin[1]);
    }else{
        s0=s_temp - sqrt(vector_match_to_origin[0]*vector_match_to_origin[0]+
                         vector_match_to_origin[1]*vector_match_to_origin[1]);
    }
    for (double &element : index2s){
        element-=s0;
    }
    return index2s;
}

double CalcSFromIndex2S(const vector<double> &index2s,const vector<double> &path_x,
                        const vector<double> &path_y,const double &proj_x,const double &proj_y, 
                        const int &proj_match_point_index){
    double vector_1[2]={proj_x-path_x[proj_match_point_index],
                        proj_y-path_y[proj_match_point_index]};
    double vector_2[2]={std::nan(""),std::nan("")};
    if (proj_match_point_index < path_x.size()-1){
        vector_2[0]=path_x[proj_match_point_index+1]-path_x[proj_match_point_index];
        vector_2[1]=path_y[proj_match_point_index+1]-path_y[proj_match_point_index];
    }else{
        vector_2[0]=path_x[proj_match_point_index]-path_x[proj_match_point_index-1];
        vector_2[1]=path_y[proj_match_point_index]-path_y[proj_match_point_index-1];        
    }
    double flag=vector_1[0]*vector_2[0] + vector_1[1]*vector_2[1];
    double ss;
    if (flag > 0){
        ss=index2s[proj_match_point_index] + sqrt(vector_1[0]*vector_1[0] + 
                                                  vector_1[1]*vector_1[1]);
    }else{
        ss=index2s[proj_match_point_index] - sqrt(vector_1[0]*vector_1[0] + 
                                                  vector_1[1]*vector_1[1]);
    }
    return ss;
}

std::tuple<vector<double>,vector<double>> Cartesian2Frenet(const vector<double> &x_set, 
                const vector<double> &y_set , const vector<double> &frenet_path_x,
            const vector<double> &frenet_path_y,const vector<Proj_Origin_Point> &proj_points,
            const vector<double> &index2s){
    vector<double> s_set(maxnum,std::nan(""));
    vector<double> l_set(maxnum,std::nan(""));
    for (int i=0;i<x_set.size();i++){
        if (std::isnan(x_set[i])) break;
        s_set[i]=CalcSFromIndex2S(index2s,frenet_path_x,frenet_path_y, 
                proj_points[i].proj_x, proj_points[i].proj_y, proj_points[i].match_index);
        double n_r[2]={-sin(proj_points[i].proj_heading),cos(proj_points[i].proj_heading)};
        double h_r[2]={x_set[i]-proj_points[i].proj_x , y_set[i]-proj_points[i].proj_y};
        l_set[i]=n_r[0]*h_r[0] + n_r[1]*h_r[1];
    }
    return std::make_tuple(s_set,l_set);
}

std::tuple<double,double,double,double> Cartesian2Frenet(const Plan_Start_gcs &plan_start, 
                const vector<double> &frenet_path_x,const vector<double> &frenet_path_y, 
                const Proj_Origin_Point &proj_point,const vector<double> &index2s){
    double s_set=CalcSFromIndex2S(index2s,frenet_path_x,frenet_path_y,proj_point.proj_x,
                                                proj_point.proj_y,proj_point.match_index);
    // std::cout<<"s_set=="<<s_set<<std::endl;
    // std::cout<<"proj_point.proj_x=="<<proj_point.proj_x<<std::endl;
    // std::cout<<"proj_point.proj_y=="<<proj_point.proj_y<<std::endl;
    // for (int i=0;i<index2s.size();i++){
    //     std::cout<<"index2s=="<<index2s[i]<<std::endl;
    // }
    double n_r[2]={-sin(proj_point.proj_heading),cos(proj_point.proj_heading)};
    double h_r[2]={plan_start.x-proj_point.proj_x , plan_start.y-proj_point.proj_y};
    double l_set=n_r[0]*h_r[0] + n_r[1]*h_r[1];

    double v_h[2]={plan_start.vx, plan_start.vy};
    double t_r[2]={cos(proj_point.proj_heading),sin(proj_point.proj_heading)};
    double l_dot_set=v_h[0]*n_r[0] + v_h[1]*n_r[1];
    double s_dot_set=(v_h[0]*t_r[0] + v_h[1]*t_r[1])/(1-proj_point.proj_kappa*l_set);
    double dl_set;
    if (abs(s_dot_set) < 1e-6){
        dl_set=0.0;
    }else{
        dl_set=l_dot_set/s_dot_set;
    }

    double a_h[2]={plan_start.ax , plan_start.ay};
    double l_dot2_set=(a_h[0]*n_r[0]+a_h[1]*n_r[1]) - 
            proj_point.proj_kappa*(1-proj_point.proj_kappa*l_set)*pow(s_dot_set,2);
    double s_dot2_set=(a_h[0]*t_r[0] + a_h[1]*t_r[1] + 
            2*proj_point.proj_kappa*dl_set*pow(s_dot_set,2))/(1-proj_point.proj_kappa*l_set);
    double ddl_set;
    if (s_dot2_set < 1e-6){
        ddl_set=0.0;
    }else{
        ddl_set=(l_dot2_set - dl_set*s_dot2_set)/(pow(s_dot_set,2));
    }
    return std::make_tuple(s_set,l_set,dl_set,ddl_set);
}

std::tuple<Plan_Start_gcs,Plan_Trajectory> Calc_Plan_Start_Info_And_Stitch_Trajectory(
                        const Plan_Trajectory &pre_trajectory, const double current_time,
                        const Host_State &host){
    static bool init_run=true;
    vector<double> stitch_x(20,0.0);
    vector<double> stitch_y(20,0.0);
    vector<double> stitch_heading(20,0.0);
    vector<double> stitch_kappa(20,0.0);
    vector<double> stitch_speed(20,0.0);
    vector<double> stitch_accel(20,0.0);
    vector<double> stitch_time(20,-1);

    Plan_Trajectory trajectory;
    Plan_Start_gcs plan_start;

    if (init_run){
        init_run=false;
        double plan_start_x=host.x;
        double plan_start_y=host.y;
        double plan_start_heading=host.heading_xy;
        double plan_start_kappa=0.0;
        double plan_start_vx=0.0;
        double plan_start_vy=0.0;
        double plan_start_ax=0.0;
        double plan_start_ay=0.0;
        double plan_start_time=current_time + PLAN_CYCLE;

        trajectory.trajectory_x=stitch_x;
        trajectory.trajectory_y=stitch_y;
        trajectory.trajectory_heading=stitch_heading;
        trajectory.trajectory_kappa=stitch_kappa;
        trajectory.trajectory_velocity=stitch_speed;
        trajectory.trajectory_accel=stitch_accel;
        trajectory.trajectory_time=stitch_time;

        plan_start.x=plan_start_x;
        plan_start.y=plan_start_x;
        plan_start.heading=plan_start_heading;
        plan_start.kappa=plan_start_kappa;
        plan_start.vx=plan_start_vx;
        plan_start.vy=plan_start_vy;
        plan_start.ax=plan_start_ax;
        plan_start.ay=plan_start_ay;
        plan_start.start_time=plan_start_time;
        return std::make_tuple(plan_start,trajectory);
    }else{
        double x_cur=host.x;
        double y_cur=host.y;
        double heading_cur=host.heading_xy;
        double kappa_cur=0.0;
        double vx_cur=host.vx*cos(heading_cur) - host.vy*sin(heading_cur);
        double vy_cur=host.vx*sin(heading_cur) + host.vy*cos(heading_cur);
        double ax_cur=host.ax*cos(heading_cur) - host.ay*sin(heading_cur);
        double ay_cur=host.ax*sin(heading_cur) + host.ay*cos(heading_cur);
        double dt=PLAN_CYCLE;
        int index;
        double min_dis=numeric_limits<double>::max();
        for (int i=0;i<pre_trajectory.trajectory_x.size();i++){
            double dis=pow(host.x-pre_trajectory.trajectory_x[i],2) + 
                        pow(host.y-pre_trajectory.trajectory_y[i],2);
            if (dis<min_dis){
                    min_dis=dis;
                    index=i;
                    break;
            }
            double pre_x_desire=pre_trajectory.trajectory_x[index];
            double pre_y_desire=pre_trajectory.trajectory_y[index];
            double pre_heading_desire=pre_trajectory.trajectory_heading[index];

            double tor[2]={cos(pre_heading_desire),sin(pre_heading_desire)};
            double nor[2]={-sin(pre_heading_desire),cos(pre_heading_desire)};
            double d_err[2]={host.x-pre_x_desire , host.y-pre_y_desire};
            double lon_err=abs(d_err[0]*tor[0] + d_err[1]*tor[1]);
            double lat_err=abs(d_err[0]*nor[0] + d_err[1]*nor[1]);
            if (lon_err >2.5 || lat_err >0.5){
                double plan_start_x=x_cur + vx_cur*dt + 0.5*ax_cur*dt*dt;
                double plan_start_y=y_cur + vy_cur*dt + 0.5*ay_cur*dt*dt;
                double plan_start_vx=vx_cur + ax_cur*dt;
                double plan_start_vy=vy_cur + ay_cur*dt;
                double plan_start_heading=atan2(plan_start_vy,plan_start_vx);
                double plan_start_ax=ax_cur;
                double plan_start_ay=ay_cur;
                double plan_start_kappa=kappa_cur;
                double plan_start_time=current_time + PLAN_CYCLE;

                trajectory.trajectory_x=stitch_x;
                trajectory.trajectory_y=stitch_y;
                trajectory.trajectory_heading=stitch_heading;
                trajectory.trajectory_kappa=stitch_kappa;
                trajectory.trajectory_velocity=stitch_speed;
                trajectory.trajectory_accel=stitch_accel;
                trajectory.trajectory_time=stitch_time;

                plan_start.x=plan_start_x;
                plan_start.y=plan_start_x;
                plan_start.heading=plan_start_heading;
                plan_start.kappa=plan_start_kappa;
                plan_start.vx=plan_start_vx;
                plan_start.vy=plan_start_vy;
                plan_start.ax=plan_start_ax;
                plan_start.ay=plan_start_ay;
                plan_start.start_time=plan_start_time;
                return std::make_tuple(plan_start,trajectory);
            }else{
                double min_diss=numeric_limits<double>::max();
                double dis=pow(host.x-pre_trajectory.trajectory_x[i],2) + 
                        pow(host.y-pre_trajectory.trajectory_y[i],2);
                if (dis<min_dis){
                        min_dis=dis;
                        index=i;
                        break;
                }
                int index;
                for (int j=0;j<pre_trajectory.trajectory_time.size()-1;j++){
                    if (pre_trajectory.trajectory_time[j] <= current_time + PLAN_CYCLE &&
                        pre_trajectory.trajectory_time[j+1] > current_time + PLAN_CYCLE){
                        index=j;
                        break;
                    }
                }
                double plan_start_x=pre_trajectory.trajectory_x[index];
                double plan_start_y=pre_trajectory.trajectory_y[index];
                double plan_start_heading=pre_trajectory.trajectory_heading[index];
                double plan_start_kappa=pre_trajectory.trajectory_kappa[index];
                double plan_start_vx=pre_trajectory.trajectory_velocity[index]
                                                        *cos(plan_start_heading);
                double plan_start_vy=pre_trajectory.trajectory_velocity[index]
                                                        *sin(plan_start_heading);

                double tor[2]={cos(plan_start_heading),sin(plan_start_heading)};
                double nor[2]={-sin(plan_start_heading),cos(plan_start_heading)};
                double a_tor[2]={pre_trajectory.trajectory_accel[index]*tor[0],
                                 pre_trajectory.trajectory_accel[index]*tor[1]};
                double a_nor[2]={pow(pre_trajectory.trajectory_velocity[index],2)*
                                    plan_start_kappa*nor[0],
                                 pow(pre_trajectory.trajectory_velocity[index],2)*
                                    plan_start_kappa*nor[1]};
                double plan_start_ax=a_tor[0] + a_nor[0];
                double plan_start_ay=a_tor[1] + a_nor[1];
                double plan_start_time=pre_trajectory.trajectory_time[index];
                std::cout<<"拼接时间点==="<<plan_start_time<<std::endl;
                //轨迹拼接
                index-=1;
                if (index >=19){
                    for (int k=0;k<20;k++){
                        stitch_x[19-k]=pre_trajectory.trajectory_x[index-k];
                        stitch_y[19-k]=pre_trajectory.trajectory_y[index-k];
                        stitch_heading[19-k]=pre_trajectory.trajectory_heading[index-k];
                        stitch_kappa[19-k]=pre_trajectory.trajectory_kappa[index-k];
                        stitch_speed[19-k]=pre_trajectory.trajectory_velocity[index-k];
                        stitch_accel[19-k]=pre_trajectory.trajectory_accel[index-k];
                        stitch_time[19-k]=pre_trajectory.trajectory_time[index-k];
                    }
                }else{
                    for (int k=0;k<=index;k++){
                        stitch_x[19-k]=pre_trajectory.trajectory_x[index-k];
                        stitch_y[19-k]=pre_trajectory.trajectory_y[index-k];
                        stitch_heading[19-k]=pre_trajectory.trajectory_heading[index-k];
                        stitch_kappa[19-k]=pre_trajectory.trajectory_kappa[index-k];
                        stitch_speed[19-k]=pre_trajectory.trajectory_velocity[index-k];
                        stitch_accel[19-k]=pre_trajectory.trajectory_accel[index-k];
                        stitch_time[19-k]=pre_trajectory.trajectory_time[index-k];
                    }
                }
                trajectory.trajectory_x=stitch_x;
                trajectory.trajectory_y=stitch_y;
                trajectory.trajectory_heading=stitch_heading;
                trajectory.trajectory_kappa=stitch_kappa;
                trajectory.trajectory_velocity=stitch_speed;
                trajectory.trajectory_accel=stitch_accel;
                trajectory.trajectory_time=stitch_time;
                
                plan_start.x=plan_start_x;
                plan_start.y=plan_start_x;
                plan_start.heading=plan_start_heading;
                plan_start.kappa=plan_start_kappa;
                plan_start.vx=plan_start_vx;
                plan_start.vy=plan_start_vy;
                plan_start.ax=plan_start_ax;
                plan_start.ay=plan_start_ay;
                plan_start.start_time=plan_start_time;
                return std::make_tuple(plan_start,trajectory);
            }
        }
    }
}




void Static_Obs::Init_Obs(){
    static_obs_x_set.resize(Static_Obs::num,std::nan(""));
    static_obs_y_set.resize(Static_Obs::num,std::nan(""));
    dynamic_obs_x_set.resize(Static_Obs::num,std::nan(""));
    dynamic_obs_y_set.resize(Static_Obs::num,std::nan(""));
    dynamic_obs_vx_set.resize(Static_Obs::num,std::nan(""));
    dynamic_obs_vy_set.resize(Static_Obs::num,std::nan(""));

    static_obs_s.clear();
    static_obs_l.clear();

    proj_points.clear();
}

void Static_Obs::Handle_Obs(const Host_State &host,const vector<Obs_State> &Obs,
                        const vector<double> reference_x,const vector<double> reference_y,
                        const vector<double> reference_heading,
                        const vector<double> reference_kappa,const vector<double> &index2s){
    auto OBS=Obs_Filter(host,Obs);
    vector<double> obs_x_set_final=std::get<0>(OBS);
    vector<double> obs_y_set_final=std::get<1>(OBS); 
    vector<double> obs_heading_set_final=std::get<2>(OBS); 
    vector<double> obs_velocity_set_final=std::get<3>(OBS);

    auto Static_And_Dynamic=OBS_Static_And_Dynamic(obs_x_set_final,obs_y_set_final,
                            obs_heading_set_final,obs_velocity_set_final);
    static_obs_x_set=std::get<0>(Static_And_Dynamic);
    static_obs_y_set=std::get<1>(Static_And_Dynamic);
    dynamic_obs_x_set=std::get<2>(Static_And_Dynamic);
    dynamic_obs_y_set=std::get<3>(Static_And_Dynamic);
    dynamic_obs_vx_set=std::get<4>(Static_And_Dynamic);
    dynamic_obs_vy_set=std::get<5>(Static_And_Dynamic); 
    Renew_Proj(reference_x,reference_y,reference_heading,reference_kappa);
    Renew_SL(reference_x,reference_y,index2s);
}

void Static_Obs::Renew_Proj(const vector<double> &reference_x,const vector<double> &reference_y, 
                const vector<double> &reference_heading,const vector<double> &reference_kappa){
    SeekObsOrigin.ImplementFind(static_obs_x_set, static_obs_y_set, 
                reference_x,reference_y, reference_heading,reference_kappa);
    proj_points=SeekObsOrigin.Get_Origin_Points();
}

void Static_Obs::Renew_SL(const vector<double> &reference_x,const vector<double> &reference_y,
                            const vector<double> &index2s){
    auto S_L=Cartesian2Frenet(static_obs_x_set, 
                static_obs_y_set , reference_x,
            reference_y,proj_points,index2s);
    static_obs_s=std::get<0>(S_L);
    static_obs_l=std::get<1>(S_L);
}

std::tuple<vector<double>,vector<double>> Static_Obs::Get_SL(){
    return std::make_tuple(static_obs_s,static_obs_l);
}

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>> Obs_Filter(
                        const Host_State &host,const vector<Obs_State> &Obs){
    int num=32;
    vector<double> obs_x_set_final(num,std::nan(""));
    vector<double> obs_y_set_final(num,std::nan(""));
    vector<double> obs_velocity_set_final(num,std::nan(""));
    vector<double> obs_heading_set_final(num,std::nan(""));
    int count=0;
    for (int i=0;i<Obs.size();i++){
        if (std::isnan(Obs[i].x)) break;
        double tor[2]={cos(host.heading_xy),sin(host.heading_xy)};
        double nor[2]={-sin(host.heading_xy),cos(host.heading_xy)};
        double vector_obs[2]={Obs[i].x-host.x , Obs[i].y-host.y};
        double lon_distance=vector_obs[0]*tor[0] + vector_obs[1]*tor[1];
        double lat_distance=vector_obs[0]*nor[0] + vector_obs[1]*nor[1];

        if (lon_distance <60 && lon_distance>-10 && lat_distance <10 && lat_distance >-10){
            obs_x_set_final[count]=Obs[i].x;
            obs_y_set_final[count]=Obs[i].y;
            obs_heading_set_final[count]=Obs[i].heading;
            obs_velocity_set_final[count]=Obs[i].velocity;
            count+=1;
        }
    }
    return std::make_tuple(obs_x_set_final,obs_y_set_final,
                            obs_heading_set_final,obs_velocity_set_final);
}

std::tuple<vector<double>,vector<double>,vector<double>,vector<double>,
    vector<double>,vector<double>> OBS_Static_And_Dynamic(const vector<double> &obs_x_set,
            const vector<double> &obs_y_set,const vector<double> &obs_heading_set,
            const vector<double> &obs_velocity_set){
    int num=32;
    vector<double> static_obs_x_set(num,std::nan(""));
    vector<double> static_obs_y_set(num,std::nan(""));
    vector<double> dynamic_obs_x_set(num,std::nan(""));
    vector<double> dynamic_obs_y_set(num,std::nan(""));
    vector<double> dynamic_obs_vx_set(num,std::nan(""));
    vector<double> dynamic_obs_vy_set(num,std::nan(""));
    int count_static=0;
    int count_dynamic=0;
    for (int i=0;i<obs_x_set.size();i++){
        if (abs(obs_velocity_set[i]) < 0.1){
            static_obs_x_set[count_static]=obs_x_set[i];
            static_obs_y_set[count_static]=obs_y_set[i];
            count_static+=1;
        }else{
            dynamic_obs_x_set[count_dynamic]=obs_x_set[i];
            dynamic_obs_y_set[count_dynamic]=obs_y_set[i];
            dynamic_obs_vx_set[count_dynamic]=obs_velocity_set[i]*cos(obs_heading_set[i]);
            dynamic_obs_vy_set[count_dynamic]=obs_velocity_set[i]*sin(obs_heading_set[i]);
            count_dynamic+=1;
        }
    }
    return std::make_tuple(static_obs_x_set,static_obs_y_set,dynamic_obs_x_set,
                           dynamic_obs_y_set,dynamic_obs_vx_set,dynamic_obs_vy_set);
}

FindObsOriginPoint::FindObsOriginPoint(){
    proj_x_set.resize(maxnum,std::nan(""));
    proj_y_set.resize(maxnum,std::nan(""));
    proj_heading_set.resize(maxnum,std::nan(""));
    proj_kappa_set.resize(maxnum,std::nan(""));
    first_run=true;
    pre_match_point_index_set.resize(maxnum,std::nan(""));
}


void FindHostOriginPoint::ImplementFind(const double host_x, const double host_y, 
                const vector<double> reference_x,const vector<double> reference_y, 
            const vector<double> reference_heading,const vector<double> reference_kappa){
    int len=reference_x.size();
    int match_index;
    std::cout<<"first_run"<<first_run<<std::endl;
    if (first_run){
        int start_search_index=0;
        int increase_count=0;
        double min_distance=numeric_limits<double>::max();
        for (int j=start_search_index; j<len; j++){
            double distance=pow(host_x-reference_x[j],2)+pow(host_y-reference_y[j],2);
            if (distance < min_distance){
                min_distance=distance;
                match_index=j;
                increase_count=0;
            }else{
                increase_count+=1;
            }
            if (increase_count>50) break;
        }
        double match_x=reference_x[match_index];
        double match_y=reference_y[match_index];
        double match_heading=reference_heading[match_index];
        double match_kappa=reference_kappa[match_index];
        double match_point[2]={match_x,match_y};
        double match_point_direction[2]={cos(match_heading),sin(match_heading)};
        
        double vector_d[2]={host_x-match_x,host_y-match_y};
        double ds=vector_d[0]*match_point_direction[0] + 
                                vector_d[1]*match_point_direction[1];
        proj_x=match_point[0] + ds*match_point_direction[0];
        proj_y=match_point[1] + ds*match_point_direction[1];
        proj_heading=match_heading + match_kappa*ds;
        proj_kappa=match_kappa;

        pre_match_point_index_set=match_index;
        pre_reference_x=reference_x;
        pre_reference_y=reference_y;
        pre_reference_heading=reference_heading;
        pre_reference_kappa=reference_kappa;

        first_run=false;
    }else{
        int start_search_index=pre_match_point_index_set;
        int increase_count=0;
        int increase_count_limit=5;
        if (pre_match_point_index_set==-1){
            start_search_index=0;
            increase_count_limit=50;
        }
        double vector_pre_match_point[2]={pre_reference_x[start_search_index],
                                          pre_reference_y[start_search_index]};
        double vector_pre_match_point_direction[2]={cos(pre_reference_heading
                [start_search_index]),sin(pre_reference_heading[start_search_index])};
        double flag=(host_x-vector_pre_match_point[0])*vector_pre_match_point_direction[0]+
                    (host_y-vector_pre_match_point[1])*vector_pre_match_point_direction[1];
        double min_distance=numeric_limits<double>::max();
        if (flag > 0.001){
            for (int j=start_search_index;j<len;j++){
                double distance=pow(host_x-reference_x[j],2) + pow(host_y-reference_y[j],2);
                if (distance < min_distance){
                    min_distance=distance;
                    match_index=j;
                    increase_count=0;
                }else{
                    increase_count+=1;
                }
                if (increase_count > increase_count_limit) break;
            }
        }else if (flag < -0.001){
            for (int j=start_search_index;j>=0;j--){
                double distance=pow(host_x-reference_x[j],2) + pow(host_y-reference_y[j],2);
                if (distance < min_distance){
                    min_distance=distance;
                    match_index=j;
                    increase_count=0;
                }else{
                    increase_count+=1;
                }
                if (increase_count > increase_count_limit) break;
            }
        }else{
            match_index=start_search_index;
        }
        //std::cout<<"match_index"<<match_index<<std::endl;
        double match_x=reference_x[match_index];
        double match_y=reference_y[match_index];
        //std::cout<<"match_x"<<match_x<<std::endl;
        //std::cout<<"match_y"<<match_y<<std::endl;
        double match_heading=reference_heading[match_index];
        //std::cout<<"match_heading"<<match_heading<<std::endl;
        double match_kappa=reference_kappa[match_index];
        double match_point[2]={match_x,match_y};
        double match_point_direction[2]={cos(match_heading),sin(match_heading)};
        
        double vector_d[2]={host_x-match_x,host_y-match_y};
        double ds=vector_d[0]*match_point_direction[0] + 
                                vector_d[1]*match_point_direction[1];
        //std::cout<<"ds"<<ds<<std::endl;             
        proj_x=match_point[0] + ds*match_point_direction[0];
        proj_y=match_point[1] + ds*match_point_direction[1];
        //std::cout<<"proj_x"<<proj_x<<std::endl;
        //std::cout<<"proj_y"<<proj_y<<std::endl;
        proj_heading=match_heading + match_kappa*ds;
        proj_kappa=match_kappa;

        pre_match_point_index_set=match_index;
        pre_reference_x=reference_x;
        pre_reference_y=reference_y;
        pre_reference_heading=reference_heading;
        pre_reference_kappa=reference_kappa;

    }

    
}


void FindObsOriginPoint::ImplementFind(const vector<double> x_set, const vector<double> y_set, 
    const vector<double> reference_x,const vector<double> reference_y, 
    const vector<double> reference_heading,const vector<double> reference_kappa){
    int len=reference_x.size();
    int match_index;
    vector<int> match_point_index_set(maxnum,std::nan(""));
    if (first_run){
        pre_reference_x=reference_x;
        pre_reference_y=reference_y;
        pre_reference_heading=reference_heading;
        pre_reference_kappa=reference_kappa;
        for (auto& element : pre_match_point_index_set){
            element=std::nan("");
        }
        for (int i=0; i<x_set.size();i++){
            if (std::isnan(x_set[i])) break;
            int start_search_index=0;
            int increase_count=0;
            double min_distance=numeric_limits<double>::max();
            for (int j=start_search_index; j<len; j++){
                double distance=pow(x_set[i]-reference_x[j],2)+pow(y_set[i]-reference_y[j],2);
                if (distance < min_distance){
                    min_distance=distance;
                    match_index=j;
                    increase_count=0;
                }else{
                    increase_count+=1;
                }
                if (increase_count>50) break;
            }
            double match_x=reference_x[match_index];
            double match_y=reference_y[match_index];
            double match_heading=reference_heading[match_index];
            double match_kappa=reference_kappa[match_index];
            double match_point[2]={match_x,match_y};
            double match_point_direction[2]={cos(match_heading),sin(match_heading)};
            
            double vector_d[2]={x_set[i]-match_x , y_set[i]-match_y};
            double ds=vector_d[0]*match_point_direction[0] + 
                                    vector_d[1]*match_point_direction[1];
            double proj_x=match_point[0] + ds*match_point_direction[0];
            double proj_y=match_point[1] + ds*match_point_direction[1];
            double proj_heading=match_heading + match_kappa*ds;
            double proj_kappa=match_kappa;

            match_point_index_set[i]=match_index;
            proj_x_set[i]=proj_x;
            proj_y_set[i]=proj_y;
            proj_heading_set[i]=proj_heading;
            proj_kappa_set[i]=proj_kappa;
        }
        first_run=false;
        pre_match_point_index_set=match_point_index_set;
        pre_x_set=x_set;
        pre_y_set=y_set;

    }else{
        for (int i=0;i<x_set.size();i++){
            int start_search_index=pre_match_point_index_set[i];
            double square_dis=pow(x_set[i]-pre_x_set[i],2) + pow(y_set[i]-pre_y_set[i],2);
            if (square_dis > 36){
                start_search_index=std::nan("");
            }
            int increase_count=0;
            int increase_count_limit=5;
            if (std::isnan(start_search_index)){
                start_search_index=0;
                increase_count_limit=50;
            }
            double vector_pre_match_point[2]={pre_reference_x[start_search_index],
                                            pre_reference_y[start_search_index]};
            double vector_pre_match_point_direction[2]={cos(pre_reference_heading
                    [start_search_index]),sin(pre_reference_heading[start_search_index])};
            double flag=(x_set[i]-vector_pre_match_point[0])*vector_pre_match_point_direction[0]+
                        (y_set[i]-vector_pre_match_point[1])*vector_pre_match_point_direction[1];
            double min_distance=numeric_limits<double>::max();
            if (flag > 0.001){
                for (int j=start_search_index;j<len;j++){
                    double distance=pow(x_set[i]-reference_x[j],2) + pow(y_set[i]-reference_y[j],2);
                    if (distance < min_distance){
                        min_distance=distance;
                        match_index=j;
                        increase_count=0;
                    }else{
                        increase_count+=1;
                    }
                    if (increase_count > increase_count_limit) break;
                }
            }else if (flag < -0.001){
                for (int j=start_search_index;j>=0;j--){
                    double distance=pow(x_set[i]-reference_x[j],2) + pow(y_set[i]-reference_y[j],2);
                    if (distance < min_distance){
                        min_distance=distance;
                        match_index=j;
                        increase_count=0;
                    }else{
                        increase_count+=1;
                    }
                    if (increase_count > increase_count_limit) break;
                }
            }else{
                match_index=start_search_index;
            }
            double match_x=reference_x[match_index];
            double match_y=reference_y[match_index];
            double match_heading=reference_heading[match_index];
            double match_kappa=reference_kappa[match_index];
            double match_point[2]={match_x,match_y};
            double match_point_direction[2]={cos(match_heading),sin(match_heading)};
            
            double vector_d[2]={x_set[i]-match_x,y_set[i]-match_y};
            double ds=vector_d[0]*match_point_direction[0] + 
                                    vector_d[1]*match_point_direction[1];
            double proj_x=match_point[0] + ds*match_point_direction[0];
            double proj_y=match_point[1] + ds*match_point_direction[1];
            double proj_heading=match_heading + match_kappa*ds;
            double proj_kappa=match_kappa;

            match_point_index_set[i]=match_index;
            proj_x_set[i]=proj_x;
            proj_y_set[i]=proj_y;
            proj_heading_set[i]=proj_heading;
            proj_kappa_set[i]=proj_kappa;
            
        }
        pre_match_point_index_set=match_point_index_set;
        pre_reference_x=reference_x;
        pre_reference_y=reference_y;
        pre_reference_heading=reference_heading;
        pre_reference_kappa=reference_kappa;
        pre_x_set=x_set;
        pre_y_set=y_set;
    }
}

vector<Proj_Origin_Point> FindObsOriginPoint::Get_Origin_Points(){
    vector<Proj_Origin_Point> origin_points;
    for (int i=0;i<maxnum;i++){
        Proj_Origin_Point Origin(proj_x_set[i], proj_y_set[i], proj_heading_set[i], 
                                proj_kappa_set[i], pre_match_point_index_set[i]);

        origin_points.push_back(Origin);
    }
    return origin_points;
}