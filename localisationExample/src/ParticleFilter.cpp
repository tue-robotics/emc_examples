#include "ParticleFilter.h"

void ParticleFilter::update(double forwardMotion, double angleMotion, measurementList measurement, World world)
{
    // Propagate ParticleFilter Samples based on observed odometry information
    ParticleFilterBase::propagateSamples(forwardMotion,angleMotion);
    // Update Likelihoods and weights of the Particles based on the observed measurements
    LikelihoodVector likelihoods = ParticleFilterBase::computeLikelihoods(measurement,world);
    
    for (int i = 0; i<ParticleFilterBase::getNumberParticles(); i++)
    {
        ParticleFilterBase::_particles[i].setWeight(likelihoods[i]);
    }
    // Normalise particle weights to 1
    ParticleFilterBase::normaliseWeights();
    // Resample
    if(needsResampling())
    {
        _resampler.resample(_particles,ParticleFilterBase::getNumberParticles());
    }

    return;
}