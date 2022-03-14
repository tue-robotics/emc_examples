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

void ParticleFilter::configureResampler(std::string algorithm, std::string resamplingScheme, double long resampleThreshold)
{
    _resampler.initaliseResampler(&(_generator),algorithm);
    _resamplingScheme = resamplingScheme;
    _resampleThreshold = resampleThreshold;
}

bool ParticleFilter::needsResampling() 
{
    if (_resamplingScheme.compare("Always") == 0)
    {
        return true;
    }
    else if (_resamplingScheme.compare("effectiveParticleThreshold") == 0)
    {
        double long sumSquaredWeights = 0;
        for (int i = 0; i<ParticleFilterBase::getNumberParticles(); i++)
        {
            double long weight_i = ParticleFilterBase::_particles[i].getWeight();
            sumSquaredWeights += weight_i*weight_i;
        }

        return (1/sumSquaredWeights) < _resampleThreshold;
    }    
    else
    {
        return true;
    }
}