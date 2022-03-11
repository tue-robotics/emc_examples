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
        //_stratified(Particles,N);
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
        // Find corresponding Particle  
        double cumSum = 0;
        for (int i = 0; i<OldParticles.size(); i++)
        {
            cumSum += OldParticles[i].getWeight();
            if (cumSum >= u)
            {
                // Add Particle to new particle list
                Particle part_i = OldParticles[i];
                part_i.setWeight(1/double(N));              //todo check wheter this is going as expected
                Particles.push_back(part_i);
                break;
            }
        }
        // Keep the bookkeeping up to date
        n+=1;
    }
    return;
}

