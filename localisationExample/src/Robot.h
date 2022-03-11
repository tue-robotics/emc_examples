#pragma once

#include<vector>
#include <math.h> 
#include <random>

#include "World.h"


typedef std::vector<double> msrmnt;
typedef std::vector<msrmnt> measurementList;

class Robot: public pltObject
{
    public:
                        Robot(Pose position, std::vector<double> params);
    measurementList     measure(World world);    
    void                move(double desired_dist,double desired_rot, World world);

    double _get_noise_sample(double mu,double sigma);    


    private:
    double _x;
    double _y;
    double _theta;
    double _std_forward;
    double _std_turn;
    double _std_meas_dist;
    double _std_meas_angl;
    std::default_random_engine _generator;

};