#pragma once

#include "Robot.h"

#include <math.h>
#include <random>

typedef double Likelihood;
typedef std::vector<Likelihood> LikelihoodVector;

// Implementation of a particle, used in the implementation in the 

class Particle : public Object
{
    public:
    // Initialise Particle either uniformly or according to a Gaussian distribution
    Particle(const World &World, const double &weight, std::default_random_engine* _generatorPtr); 
    Particle(const World &World, const double mean[3], const double sigma[3], const double &weight, std::default_random_engine* _generatorPtr);
    // Replace position and weight of the Particle
    void setParticle(Particle particle);
    
    // Todo cyclic world assumption

    // Get current Position of Particle
    Pose getPosition() const;
     // Get current Weight of Particle
    Likelihood getWeight () const;
    // Set Weight of Particle
    void setWeight(const Likelihood &weight);
    // Propagate the sample based on the received odomotry information
    void propagateSample(const double &forwardMotion, const double &angleMotion, const double proc_noise[2]);
    // Compute the likelihood of the particle based on the received measurement information
    Likelihood computeLikelihood(const measurementList &measurement, const World &world, const double meas_noise[2]);   
    
    // Internal functions to compute uniform or gaussian samples
    double _get_noise_sample_unfiorm(double minval, double maxval) const;
    double _get_noise_sample_gaussian(double mu, double sigma) const;        

    // Debugging print sample inormation to terminal
    void print(const int &i) const;                

    private:
    double _x;
    double _y;
    double _theta;

    Likelihood _weight;

    std::default_random_engine* _generatorPtr;
};
