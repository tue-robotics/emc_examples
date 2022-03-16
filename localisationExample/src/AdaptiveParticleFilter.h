#pragma once

#include "ParticleFilterBase.h"
#include "Resampler.h"

#include <math.h>
#include <set>

typedef std::vector<double> Bin;
typedef std::vector<Bin> BinList;

// Implementation of an Adaptive Particle Filter using sum of likelihoods sampling

class AdaptiveParticleFilter : public ParticleFilterBase
{
    public:

    AdaptiveParticleFilter(World &World, int &N): ParticleFilterBase(World, N), _N_max(N) {}

    AdaptiveParticleFilter(World &world, double mean[3], double sigma[3], int &N): ParticleFilterBase(world, mean, sigma , N), _N_max(N) {}

    void configureAdaptive(std::string resamplingScheme, Likelihood resampleThreshold);
    
    bool needsResampling() {return true;};

    void update(const double &forwardMotion, const double &angleMotion, const measurementList &measurement, const World &world) override;

    private:

    bool idxInBins(Bin &Indices, BinList &binsWithSupport);
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
