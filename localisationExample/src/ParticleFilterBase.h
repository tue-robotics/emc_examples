#pragma once

#include<vector>
#include<string>
#include<iostream>
#include<algorithm>

#include "Robot.h"
#include "Particle.h"

typedef std::vector<Particle> ParticleList;

class ParticleFilterBase
{
    public:
    // Create empty object
    ParticleFilterBase(){}
    // Initialize Uniformly 
    ParticleFilterBase(World World, int N); 

    // Initialize based on a gaussian with mean mu and variance sigma
    ParticleFilterBase(World World, double mean[3], double sigma[3], int N);

    // Initialize noise Parameters
    void setNoiseLevel(double motion_forward_std, double motion_turn_std, double meas_dist_std, double meas_angl_std);    
    // Todo cyclic world assumption

    // Obtain mean of samples
    Pose get_average_state();

    // Get Particle Pose List
    PoseList get_PositionList() const;    

    // Propagate Samples based on odometry information
    void propagateSamples(double forwardMotion, double angleMotion);

    // Calculate the likelihood of a sample based on the obtained landmark measurements
    LikelihoodVector computeLikelihoods(measurementList measurement, World world);   

    // Calculate the max weight
    double findMaxWeight();

    // Normalise particle Weights to one.
    void normaliseWeights();

    // Pure Virtual update function, needs to be implemented by specific flavor of Particle filter
    virtual void update(double forwardMotion, double angleMotion, measurementList measurement, World world)=0;
    
    // Virtual function to initialise the resampler when necessary
    virtual void configureResampler(std::string algorithm, std::string resamplingScheme, double long resampleThreshold){}

    // Virtual function to initialise the adaptive parameters when necessary
    virtual void configureAdaptive(std::string resamplingScheme, Likelihood resampleThreshold, Likelihood likelihoodThreshold){}

    // Get number of Particles
    int getNumberParticles() const;

    // Reset number of Particles
    void resetNumberParticles();

    // Debugging Tools 
    void printAllParticles();

    std::default_random_engine _generator;
    ParticleList _particles;            // Storage of samples
    double _processNoise[2];            // Array Containing ProcessNoise values
    double _measurmentNoise[2];         // Array Containing MeasurementNoise values

    private:
    int    _N;                          // Total number of samples.
};
