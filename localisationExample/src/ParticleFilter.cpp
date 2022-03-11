#include "ParticleFilter.h"

void ParticleFilter::update(double forwardMotion, double angleMotion, measurementList measurement, World world)
{
    ParticleFilterBase::propagateSamples(forwardMotion,angleMotion);
    LikelihoodVector likelihoods = ParticleFilterBase::computeLikelihoods(measurement,world);

    for (int i = 0; i<ParticleFilterBase::getNumberParticles(); i++)
    {
        ParticleFilterBase::_particles[i].setWeight(likelihoods[i]);
    }

    ParticleFilterBase::normaliseWeights();

    if(needsResampling())
    {
        _resampler.resample(_particles,ParticleFilterBase::getNumberParticles());
    }

    return;
}