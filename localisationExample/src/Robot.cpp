#include "Robot.h"

Robot::Robot(Pose position, std::vector<double> params) : Object(position)
{
    _x     = position[0];
    _y     = position[1];
    _theta = position[2];

    _std_forward = params[0];
    _std_turn = params[1];
    _std_meas_dist = params[2];
    _std_meas_angl = params[3];
}

void Robot::move(double desired_dist,double desired_rot, World world)
{
    // Get a random sample to determine the amount to drive and turn
    double _distance_driv = _get_noise_sample(desired_dist, _std_forward);
    double _angle_driv    = _get_noise_sample(desired_rot, _std_turn);

    // Move and turn the robot 
    _theta += _angle_driv;
    _x += _distance_driv * std::cos(_theta);
    _y += _distance_driv * std::sin(_theta);

    // Odometry is a random variable centered at the desired distance
    _odom_distance = _get_noise_sample(desired_dist, _std_forward);
    _odom_angle = _get_noise_sample(desired_rot, _std_turn);

    // Bound the orientation of the robot between [-pi,pi]
    if(_theta > M_PI)
    {
        _theta -= 2* M_PI;
    }
    else if(_theta < - M_PI)
    {
        _theta += 2* M_PI;
    }

    // Set the position of the robot
    Object::setPosition({_x,_y,_theta});
}

measurementList Robot::measure(World world)
{
    // Get a list of all landmarks
    LandMarkList lms = world.getLandMarks();
    // Intialize the measurment vector
    measurementList measurement;
    measurement.reserve(lms.size());

    // Do for all measurements
    for (int i = 0; i< lms.size(); i++)
    {
        double dx = _x - lms[i].getPosition()[0];       // X offset between robot and landmark
        double dy = _y - lms[i].getPosition()[1];       // Y offset between robot and landmark

        // Actually measured distance are gaussian random variables centered at their actual vaulue
        double z_dist = _get_noise_sample(std::sqrt(dx*dx + dy*dy),_std_meas_dist);
        double z_angl = _get_noise_sample(std::atan2(dy,dx), _std_meas_angl);

        // Store the measurement in a vector to push back onto the measurement vector
        msrmnt msri = {z_dist,z_angl};
        measurement.push_back(msri);
    }

    return measurement;
}

double Robot::_get_noise_sample(double mu,double sigma)
{
    std::normal_distribution<double> distribution(mu,sigma);
    return distribution(_generator);
}
