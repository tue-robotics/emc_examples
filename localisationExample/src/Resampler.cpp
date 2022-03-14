#include "Resampler.h"

void Resampler::initaliseResampler(std::default_random_engine* generatorPtr, std::string algorithm)
{
    _generatorPtr = generatorPtr;
    _algorithm = algorithm;
}

void Resampler::resample(ParticleList &Particles, int N)
{
    if (_algorithm.compare("Multinomial") == 0)
    {
        _multinomial(Particles,N);
    }
    else if (_algorithm.compare("Stratified") == 0)
    {
        _stratified(Particles,N);
    }
    else
    {
        _multinomial(Particles,N);
    }
}

void Resampler::_multinomial(ParticleList &Particles, int N)
{
    int n = 0;
    ParticleList OldParticles = Particles;    
    Particles.clear();

    while (n < N)
    {
        // Draw random sample
        std::uniform_real_distribution<double> distribution(1e-6,1);
        double u = distribution(*_generatorPtr);
        // Compute Cumulative Sum Vector of likelihoods
        std::vector<double long> Q;
        Q.reserve(OldParticles.size());
        long double runningVar = 0;
        for(int i = 0; i<OldParticles.size(); i++)
        {
            runningVar += OldParticles[i].getWeight();
            Q.push_back(runningVar);
        }
        // Find corresponding Particle  
        int m = 0;
        while ( Q[m]<u ) 
        {
            m++;
        }
        // Add Particle to Resampled set of particles
        Particle part_m = OldParticles[m];
        part_m.setWeight(1/double(N));
        Particles.push_back(part_m);
        // Keep the bookkeeping up to date
        n+=1;
    }
    return;
}

void Resampler::_stratified(ParticleList &Particles, int N)
{





}