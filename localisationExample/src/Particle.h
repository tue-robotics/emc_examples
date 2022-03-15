#pragma once

#include "Robot.h"

#include <math.h>
#include <random>

typedef double Likelihood;
typedef std::vector<Likelihood> LikelihoodVector;

class Particle : public pltObject
{
    public:
    // Initialise Particle either uniformly or according to a Gaussian distribution
    Particle(World World,double weight, std::default_random_engine* _generatorPtr); 
    Particle(World World, double mean[3], double sigma[3],double weight, std::default_random_engine* _generatorPtr);
    // Replace position and weight of the Particle
    void setParticle(Particle particle);
    
    // Todo cyclic world assumption

    // Get current Position of Particle
    Pose getPosition() const;
     // Get current Weight of Particle
    Likelihood getWeight () const;
    // Set Weight of Particle
    void setWeight(Likelihood weight);
    // Propagate the sample based on the received odomotry information
    void propagateSample(double forwardMotion, double angleMotion, double proc_noise[2]);
    // Compute the likelihood of the particle based on the received measurement information
    Likelihood computeLikelihood(measurementList measurement, World world, double meas_noise[2]);   
    
    // Internal functions to compute uniform or gaussian samples
    double _get_noise_sample_unfiorm(double minval, double maxval);
    double _get_noise_sample_gaussian(double mu,double sigma);        

    // Debugging print sample inormation to terminal
    void print(int i);                

    private:
    double _x;
    double _y;
    double _theta;

    Likelihood _weight;

    std::default_random_engine* _generatorPtr;
};
