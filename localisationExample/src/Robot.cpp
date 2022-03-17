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
    _distance_driv = _get_noise_sample(desired_dist, _std_forward);
    _angle_driv    = _get_noise_sample(desired_rot, _std_turn);

    _theta += _angle_driv;
    _x += _distance_driv * std::cos(_theta);
    _y += _distance_driv * std::sin(_theta);

    if(_theta > M_PI)
    {
        _theta -= 2* M_PI;
    }
    else if(_theta < - M_PI)
    {
        _theta += 2* M_PI;
    }

    // Todo cyclic world assumption
    Object::setPosition({_x,_y,_theta});
}

measurementList Robot::measure(World world)
{
    LandMarkList lms = world.getLandMarks();

    measurementList measurement;

    for (int i = 0; i< lms.size(); i++)
    {
        double dx = _x - lms[i].getPosition()[0];
        double dy = _y - lms[i].getPosition()[1];

        double z_dist = _get_noise_sample(std::sqrt(dx*dx + dy*dy),_std_meas_dist);
        double z_angl = _get_noise_sample(std::atan2(dy,dx), _std_meas_angl);

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
