#pragma once

#include "ParticleFilterBase.h"
#include "Resampler.h"

class ParticleFilter : public ParticleFilterBase
{
    public:

    ParticleFilter(World World, int N): ParticleFilterBase(World, N) {}

    ParticleFilter(World world, double mean[3], double sigma[3], int N): ParticleFilterBase(world, mean, sigma , N) {}

    void configureResampler(std::string algorithm, std::string resamplingScheme, double long resampleThreshold);
    
    bool needsResampling();

    void update(double forwardMotion, double angleMotion, measurementList measurement, World world);

    void printAllParticles() {ParticleFilterBase::printAllParticles();}

    private:
    Resampler _resampler;
    std::string _resamplingScheme;
    Likelihood _resampleThreshold;
};
