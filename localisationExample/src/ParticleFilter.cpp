#include "ParticleFilter.h"

void ParticleFilter::update(const double &forwardMotion, const double &angleMotion, const measurementList &measurement, const World &world) 
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

bool ParticleFilter::needsResampling() const
{
    if (_resamplingScheme.compare("Always") == 0)
    {
        return true;
    }
    else if (_resamplingScheme.compare("effectiveParticleThreshold") == 0)
    {
        // Lambda to acummulate the squared of all particle weights
        auto accumulateSquaredWeight = [](Likelihood i, const Particle& o){return i + o.getWeight()*o.getWeight();};
        // Comupte the sum of the squared particle weights
        Likelihood sumSquaredWeights = std::accumulate(begin(_particles), 
                                                       end(_particles), 
                                                       Likelihood(0.0), 
                                                       accumulateSquaredWeight);
        // Return true when the threshold is met
        return (1/sumSquaredWeights) < _resampleThreshold*ParticleFilterBase::getNumberParticles();
    }    
    else
    {
        return true;
    }
}
