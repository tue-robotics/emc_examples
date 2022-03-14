#pragma once

#include "ParticleFilterBase.h"
#include "Resampler.h"

class ParticleFilter : public ParticleFilterBase
{
    public:

    ParticleFilter(World World, int N): ParticleFilterBase(World, N)
    { 
        _resampler.initaliseResampler(&(_generator),"Multinomial");     
    }

    ParticleFilter(World world, double mean[3], double sigma[3], int N): ParticleFilterBase(world, mean, sigma , N)
    { 
        _resampler.initaliseResampler(&(_generator),"Multinomial");     
    }
    
    bool needsResampling() {return true;}

    void update(double forwardMotion, double angleMotion, measurementList measurement, World world);

    void printAllParticles() {ParticleFilterBase::printAllParticles();}

    private:
    Resampler _resampler;
};