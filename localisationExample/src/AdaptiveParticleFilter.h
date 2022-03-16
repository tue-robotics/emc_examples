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

    AdaptiveParticleFilter(World World, int N): ParticleFilterBase(World, N), _N_max(N) {}

    AdaptiveParticleFilter(World world, double mean[3], double sigma[3], int N): ParticleFilterBase(world, mean, sigma , N), _N_max(N) {}

    void configureAdaptive(std::string resamplingScheme, Likelihood resampleThreshold, Likelihood likelihoodThreshold);
    
    bool needsResampling() {return true;};

    void update(double forwardMotion, double angleMotion, measurementList measurement, World world) override;

    void printAllParticles() {ParticleFilterBase::printAllParticles();}

    private:

    bool idxInBins(Bin &Indices, BinList &binsWithSupport);
    double  computeRequiredParticles(int Nbins, double _epsilon, double _upperQ);

    Resampler _resampler;
    std::string _resamplingScheme;
    Likelihood _resampleThreshold;

    Likelihood _likelihoodThreshold;

    //Configuration Parameters
    std::vector<double> _resolution;
    double _epsilon;
    double _upperQ;
    int _N_min;  
    int _N_max;
    
};
