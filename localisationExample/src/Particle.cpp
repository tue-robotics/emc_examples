#include "Particle.h"

#define PI 3.141

Particle::Particle(World world, double weight, std::default_random_engine* generatorPtr) : pltObject()
{
    _generatorPtr = generatorPtr;

    _x = _get_noise_sample_unfiorm(0,world._size_x);
    _y = _get_noise_sample_unfiorm(0,world._size_y);
    _theta = _get_noise_sample_unfiorm(0,2*PI);

    _weight = weight;
}

Particle::Particle(World World, double mean[3], double sigma[3], double weight, std::default_random_engine* generatorPtr) : pltObject()
{
    _generatorPtr = generatorPtr;

    _x = _get_noise_sample_gaussian(mean[0],sigma[0]);
    _y = _get_noise_sample_gaussian(mean[1],sigma[1]);
    _theta = _get_noise_sample_gaussian(mean[2],sigma[2]);

    _weight = weight;

    Pose particlePose = {_x,_y,_theta};
    pltObject::setPosition(particlePose);
}


double Particle::_get_noise_sample_gaussian(double mu,double sigma)
{
    std::normal_distribution<double> distribution(mu,sigma);
    return distribution(*_generatorPtr);
}

double Particle::_get_noise_sample_unfiorm(double minval, double maxval)
{
    std::uniform_real_distribution<double> distribution(minval,maxval);
    return distribution(*_generatorPtr);
}

Pose Particle::getPosition()
{
    return {_x,_y,_theta};
}

void Particle::setWeight(double weight)
{
    _weight = weight;
}

double Particle::getWeight()
{
    return _weight;
}

void Particle::propagateSample(double forwardMotion, double angleMotion, double proc_noise[2])
{
    _theta += _get_noise_sample_gaussian(angleMotion, proc_noise[1]);    
    double displacement = _get_noise_sample_gaussian(forwardMotion, proc_noise[0]);
        
    _x += displacement * std::cos(_theta);
    _y += displacement * std::sin(_theta);

    Pose particlePose = {_x,_y,_theta};
    pltObject::setPosition(particlePose);
}

double long Particle::computeLikelihood(measurementList measurement, World world, double meas_noise[2])
{
    LandMarkList lms = world.getLandMarks();

    double likelihoodSample = 1.0;

    for (int i = 0; i<lms.size(); i++)
    {
        Pose lmPose = lms[i].getPosition();
        double dx = _x - lmPose[0];
        double dy = _y - lmPose[1];

        double distExpected = std::sqrt(dx*dx + dy*dy);
        double anglExpected = std::atan2(dy, dx);

        if (anglExpected> PI)
        {
            anglExpected -= 2*PI;
        }
        else if (anglExpected < -PI)
        {
            anglExpected += 2*PI;
        }

        double deltaDist = distExpected - measurement[i][0];
        double deltaAngl = anglExpected - measurement[i][1];
        
        double probMeasGivenDist = std::exp(-(deltaDist) * (deltaDist) / ( 2 * meas_noise[0] * meas_noise[0]));
        double probMeasGivenAngl = std::exp(-(deltaAngl * deltaAngl) / ( 2 * meas_noise[1] * meas_noise[1]));

        likelihoodSample *= probMeasGivenAngl * probMeasGivenDist;
    }

    return likelihoodSample;
}

void Particle::print(int i)
{
    std::cout << "-- Particle "<< i <<" -- " << std::endl;
    std::cout << " X: "<< _x << " Y: "<< _y << " Th: "<< _theta<< " Weight: "<< _weight << std::endl;
}

