#include "Resampler.h"

void Resampler::initaliseResampler(std::default_random_engine* generatorPtr, std::string algorithm)
{
    _generatorPtr = generatorPtr;
    _algorithm = algorithm;
}

void Resampler::resample(ParticleList &Particles, int N)
{   
    // Selects the desired resampling algorithm
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

LikelihoodVector Resampler::_makeCumSumVector(const ParticleList &Particles)
{
    LikelihoodVector Q; // Create Empty vector
    Q.reserve(Particles.size()); // Reserve space for N elements
    long double runningSum = 0; // Running sum of particle 0 till i 

    for(int i = 0; i<Particles.size(); i++)
    {
        runningSum += Particles[i].getWeight(); // Add the weight of particle i to the running sum
        Q.push_back(runningSum); // Append the running sum
    }
    return Q;
} 

void Resampler::_multinomial(ParticleList &Particles, int N)
{
    // Counters
    int n = 0;
    
    // Clear current Particles, save old particles
    ParticleList OldParticles = Particles;    
    Particles.clear();

    // Compute Cumulative Sum Vector of weights
    LikelihoodVector Q = _makeCumSumVector(OldParticles);

    while (n < N)
    {
        // Draw random sample
        std::uniform_real_distribution<double> distribution(1e-6,1);
        double u = distribution(*_generatorPtr);
        // Find corresponding Particle  
        auto islarger = [u](Likelihood i){return i>=u;};        // Lambda function, captures parameter u, receives from find the current weight value, 
                                                                // and returns a bool corresponding to the condition
        auto it = std::find_if(begin(Q),end(Q),islarger);       // Find the Particle with weight higher than u

        int  m  = it-Q.begin();                                 // Retrieve Particle Index from iterator
        // Add Particle to Resampled set of particles
        Particle part_m = OldParticles[m];
        part_m.setWeight(1.0/N);
        Particles.push_back(part_m);
        // Keep the bookkeeping up to date
        n+=1;
    }
    return;
}

void Resampler::_stratified(ParticleList &Particles, int N)
{
    // Counters
    int n = 0;
    int m = 0;

    // Clear current Particles, save old particles
    ParticleList OldParticles = Particles;    
    Particles.clear();

    // Compute Cumulative Sum Vector of weights
    LikelihoodVector Q = _makeCumSumVector(OldParticles);

    while (n < N)
    {
        // Draw random sample
        std::uniform_real_distribution<double> distribution(1e-10,1/N);
        double u0 = distribution(*_generatorPtr);
        double u  = u0 + double(n)/double(N);
        // Find corresponding Sample
        auto islarger = [u](Likelihood i){return i>=u;};       // Lambda function, captures parameter u, receives from find the current weight value, 
                                                               // and returns a bool corresponding to the condition
        auto it = std::find_if(begin(Q) + m,end(Q),islarger);  // Find the Particle with weight higher than u, start searching from m
        
        m  = it-Q.begin();                                     // Retrieve Particle Index from iterator
        // Append Particle to Set of new particles
        Particle particleM = OldParticles[m];
        particleM.setWeight(1.0/N);
        Particles.push_back(particleM);
        // Keep Bookkeeping up to date
        n++;
    }
    return;
}

int Resampler::generateSampleIndex(ParticleList &Particles)
{
    // This functions samples a particle index based on the 
    // weights of the respective particles

    // Generate a vector containing the cumulative weight of the particles
    //(Would be more efficient to only calculate this once every time the resampler is called)
    LikelihoodVector Q = _makeCumSumVector(Particles); 
    
    // Sample a unif-dist. random variable, between 0 and 1
    std::uniform_real_distribution<double> distribution(1e-6,1);
    double u = distribution(*_generatorPtr);

    // Lambda that determines wheter the element of the cum.sum is larger than the sample
    auto islarger = [u](Likelihood i){return i>=u;};
    // Find the first particle that statisfies the above
    auto it = std::find_if(begin(Q),end(Q),islarger);
    // Obtain idx from iterator 
    int  m  = it-Q.begin();
    // Return this index
    return m;
}