#pragma once

#include <vector>
#include <math.h> 
#include <random>

#include "World.h"


typedef std::vector<double> msrmnt;
typedef std::vector<msrmnt> measurementList;

class Robot: public Object
{
    public:
    // Initialise the robot at a certain position, and given the noise parameters
    Robot(Pose position, std::vector<double> params);
    
    // Measure the distance and angle to the landmarks from the current position
    measurementList measure(World world);    

    // Move the desired distance and turn the desired amount 
    void move(double desired_dist,double desired_rot, World world);

    // Get a gaussian noise sample in order to simulate noisy measurments and odometry
    double _get_noise_sample(double mu,double sigma);    

    double _odom_distance;
    double _odom_angle;
    
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
