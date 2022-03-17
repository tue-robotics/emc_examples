#pragma once

#include "ParticleFilterBase.h"
#include "Resampler.h"

// Implementation of an Conventional Particle Filter using two types of resampling and two types of ressampling scheme
// For a theoretical background the reader is reffered to the original paper on which this code is based:
// 
// Elfring J, Torta E, van de Molengraft R. Particle Filters: A Hands-On Tutorial. Sensors. 2021; 21(2):438.

class ParticleFilter : public ParticleFilterBase
{
    public:
    // Constructor that constructs the particle filter, and uses a uniform distribution of particles
    ParticleFilter(World &World, int &N): ParticleFilterBase(World, N) {}
    // Constructor that constructs the particle filter, and uses a gaussian distribution of particles
    ParticleFilter(World &world, double mean[3], double sigma[3], int &N): ParticleFilterBase(world, mean, sigma , N) {}

    // Configure the resampler of the conventional particle filter
    void configureResampler(std::string algorithm, std::string resamplingScheme, double long resampleThreshold);
    
    // Decide whether resampling is necessary given the given resamplingScheme
    bool needsResampling() const;

    // Update the position, and likelihood of all particles, given the measurement and odometry.
    // Furthermore resample when necessary
    void update(const double &forwardMotion, const double &angleMotion, const measurementList &measurement, const World &world) override;

    private:
    Resampler _resampler;
    std::string _resamplingScheme;
    Likelihood _resampleThreshold;
};
