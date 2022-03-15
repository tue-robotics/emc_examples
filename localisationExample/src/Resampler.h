#pragma once
#include "ParticleFilterBase.h"

class Resampler
{
        public:
    void initaliseResampler (std::default_random_engine* generatorPtr, std::string algorithm); 
    void resample(ParticleList &Particles, int N);

    private:
     // Construct Vector Q, containing the cummulative sum of the Particle Weights
    LikelihoodVector _makeCumSumVector(const ParticleList &Particles);                 
    // Implements Multinomial Resampling
    void _multinomial(ParticleList &Particles, int N);
    // Implements Stratified Resampling
    void _stratified(ParticleList &Particles, int N);

    std::string _algorithm;
    std::default_random_engine* _generatorPtr;
};