#pragma once

#include "ParticleFilterBase.h"
#include "Resampler.h"

#include <math.h>
#include <set>

typedef std::vector<double> Bin;
typedef std::vector<Bin> BinList;

// Implementation of an Adaptive Particle Filter using kld-resampling
// For a theoretical background the reader is reffered to the original paper on which this code is based:
// 
// Elfring J, Torta E, van de Molengraft R. Particle Filters: A Hands-On Tutorial. Sensors. 2021; 21(2):438.

class AdaptiveParticleFilter : public ParticleFilterBase
{
    public:
    
    // Initiate the particle filter either uniformly across the world or based on a gaussian with mean and sigma. 
    // N denotes the number of particles at t=0
    AdaptiveParticleFilter(World &World, int &N): ParticleFilterBase(World, N), _N_max(N) {}
    AdaptiveParticleFilter(World &world, double mean[3], double sigma[3], int &N): ParticleFilterBase(world, mean, sigma , N), _N_max(N) {}

    // Configure the resampling of the adaptive particle filter
    // Note: the resampling algorithm parameter in the json file is ignored
    void configureAdaptive(std::string resamplingScheme, Likelihood resampleThreshold);
    
    // Determines whether resampling is currently necessary
    bool needsResampling() {return true;};

    // Update the position, and likelihood of all particles, given the measurement and odometry.
    // Furthermore resample when necessary
    void update(const double &forwardMotion, const double &angleMotion, const measurementList &measurement, const World &world) override;

    private:

    // Find whether the indices are already in the list of bins with support
    bool idxInBins(Bin &Indices, BinList &binsWithSupport);

    // Compute the number of required particles given the number of bins and the config-parameters
    double computeRequiredParticles(int Nbins, double _epsilon, double _upperQ);

    Resampler _resampler;
    std::string _resamplingScheme;
    Likelihood _resampleThreshold;

    //Configuration Parameters
    std::vector<double> _resolution;
    double _epsilon;
    double _upperQ;
    int _N_min;  
    int _N_max;
    
};
