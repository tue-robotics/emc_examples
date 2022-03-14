#pragma once
#include "ParticleFilterBase.h"

class Resampler
{
        public:
    void initaliseResampler (std::default_random_engine* generatorPtr, std::string algorithm); 
    void resample(ParticleList &Particles, int N);

    private:

    void _multinomial(ParticleList &Particles, int N);
    void _stratified(ParticleList &Particles, int N);

    std::string _algorithm;
    std::default_random_engine* _generatorPtr;
};