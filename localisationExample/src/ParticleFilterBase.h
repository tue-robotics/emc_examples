#pragma once
#include "Robot.h"
#include "Particle.h"

typedef std::vector<Particle> ParticleList;
typedef std::vector<double long> LikelihoodVector;

class ParticleFilterBase
{
    public:
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
    PoseList get_PositionList();

    // Propagate Samples based on odometry information
    void propagateSamples(double forwardMotion, double angleMotion);

    // Calculate the likelihood of a sample based on the obtained landmark measurements
    LikelihoodVector computeLikelihoods(measurementList measurement, World world);   

    // Calculate the max weight
    double findMaxWeight();

    // Normalise particle Weights to one.
    void normaliseWeights();

    // Virtual update function, needs to be implemented by specific flavor of Particle filter
    virtual void update() {}

    // Get number of Particles
    int getNumberParticles();

    // Debugging Tools 
    void printAllParticles();
    void plotAllParticles();

    std::default_random_engine _generator;
    ParticleList _particles;            // Storage of samples

    private:
    double _processNoise[2];
    double _measurmentNoise[2];
    int    _N;                          // Total number of samples.
};