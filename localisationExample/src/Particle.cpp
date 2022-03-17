#include "Particle.h"

Particle::Particle(const World &world, const double &weight, std::default_random_engine* generatorPtr) : Object()
{
    // Store a pointer to the generator of the particle filter
    _generatorPtr = generatorPtr;
    // Generate a unif.dist. random pose in the world
    _x = _get_noise_sample_unfiorm(0,world._size_x);
    _y = _get_noise_sample_unfiorm(0,world._size_y);
    _theta = _get_noise_sample_unfiorm(0,2*M_PI);
    // Set the weight of the particle
    _weight = weight;

    Pose particlePose = {_x,_y,_theta};
    Object::setPosition(particlePose);
}

Particle::Particle(const World &World, const double mean[3], const double sigma[3], const double &weight, std::default_random_engine* generatorPtr) : Object()
{
    // Store a pointer to the generator of the particle filter
    _generatorPtr = generatorPtr;
    // Generate a gaussian.dist. random pose in the world
    _x = _get_noise_sample_gaussian(mean[0],sigma[0]);
    _y = _get_noise_sample_gaussian(mean[1],sigma[1]);
    _theta = _get_noise_sample_gaussian(mean[2],sigma[2]);
    // Set the weight of the particle
    _weight = weight;

    Pose particlePose = {_x,_y,_theta};
    Object::setPosition(particlePose);
}


double Particle::_get_noise_sample_gaussian(double mu,double sigma) const
{
    std::normal_distribution<double> distribution(mu,sigma);
    return distribution(*_generatorPtr);
}

double Particle::_get_noise_sample_unfiorm(double minval, double maxval) const
{
    std::uniform_real_distribution<double> distribution(minval,maxval);
    return distribution(*_generatorPtr);
}

Pose Particle::getPosition() const
{
    return {_x,_y,_theta};
}

void Particle::setWeight(const Likelihood &weight)
{
    _weight = weight;
}

Likelihood Particle::getWeight() const
{
    return _weight;
}

void Particle::propagateSample(const double &forwardMotion, const double &angleMotion, const double proc_noise[2])
{
    // Incorporate the odom information to change the location of the particles
    // Sample a new angle theta given the odometry information and magnitude of the noise
    _theta += _get_noise_sample_gaussian(angleMotion, proc_noise[1]);               

    // Sample a displacement given the odometry information and magnitude of the noise
    double displacement = _get_noise_sample_gaussian(forwardMotion, proc_noise[0]);
        
    // The new x and y location is obtained by using the displacement and new theta angle
    // Assuming that the robot first turns then drives
    _x += displacement * std::cos(_theta);
    _y += displacement * std::sin(_theta);

    Pose particlePose = {_x,_y,_theta};
    Object::setPosition(particlePose);
}

Likelihood Particle::computeLikelihood(const measurementList &measurement, const World &world, const double meas_noise[2])
{
    // Incorporate the measurment information to compute the likelihood of a particle
    LandMarkList lms = world.getLandMarks();

    // the likelihood is implmented recursively.
    // The likelihood of the paricle is 
    // p(particle | measurements) = product over p(particle | measurement_i)
    // i.e. measurments are assumed to be independent of eachother
    double likelihoodSample = 1.0;
    for (int i = 0; i<lms.size(); i++)
    {
        Pose lmPose = lms[i].getPosition();     // Position of landmark i
        double dx = _x - lmPose[0]; // Change in x-location between particle and landmark 
        double dy = _y - lmPose[1]; // Change in y-location between particle and landmark

        double distExpected = std::sqrt(dx*dx + dy*dy);     // Expected distance given particle and map
        double anglExpected = std::atan2(dy, dx);           // Expected angle given particle and map

        // Bound the angle between [-pi,pi]
        if (anglExpected> M_PI)
        {
            anglExpected -= 2*M_PI;
        }
        else if (anglExpected < -M_PI)
        {
            anglExpected += 2*M_PI;
        }

        // Difference between measurement and expectation
        double deltaDist = distExpected - measurement[i][0];
        double deltaAngl = anglExpected - measurement[i][1];
        
        // Calculate the likelihood of a measurement given the expectation and the amount of noise, according to a gaussian model
        double probMeasGivenDist = std::exp(-(deltaDist) * (deltaDist) / ( 2 * meas_noise[0] * meas_noise[0]));
        double probMeasGivenAngl = std::exp(-(deltaAngl * deltaAngl) / ( 2 * meas_noise[1] * meas_noise[1]));
        // Recursivley add the landmarks to the overall likelihood of the particle
        likelihoodSample *= probMeasGivenAngl * probMeasGivenDist;
    }

    return likelihoodSample;
}

void Particle::print(const int &i) const
{
    std::cout << "-- Particle "<< i <<" -- " << std::endl;
    std::cout << " X: "<< _x << " Y: "<< _y << " Th: "<< _theta<< " Weight: "<< _weight << std::endl;
}
