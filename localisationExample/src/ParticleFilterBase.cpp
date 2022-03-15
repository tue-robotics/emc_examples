#include "ParticleFilterBase.h"

ParticleFilterBase::ParticleFilterBase(World world, int N)
{
    _N = N;
    Likelihood weight = 1.0/double(N);  
    for (int i = 0; i<_N; i++)
    {
        Particle particle_i(world, weight, &_generator);
        _particles.push_back(particle_i);
    }
}


ParticleFilterBase::ParticleFilterBase(World world, double mean[3], double sigma[3], int N)
{
    _N = N;
    Likelihood weight = 1/_N;    
    for (int i = 0; i<_N; i++)
    {  
        Particle particle_i(world, mean, sigma, weight, &_generator);
        _particles.push_back(particle_i);
    }
}

void ParticleFilterBase::setNoiseLevel(double motion_forward_std, double motion_turn_std, double meas_dist_std, double meas_angl_std)
{   
    _processNoise[0] = motion_forward_std;
    _processNoise[1] = motion_turn_std;

    _measurmentNoise[0] = meas_dist_std;
    _measurmentNoise[1] = meas_angl_std;
}

void ParticleFilterBase::propagateSamples(double forwardMotion, double angleMotion)
{
    for (int i = 0; i<_N; i++)
    {
        _particles[i].propagateSample(forwardMotion, angleMotion, _processNoise);
    }
}

LikelihoodVector ParticleFilterBase::computeLikelihoods(measurementList measurement, World world)
{
    LikelihoodVector result;
    result.reserve(_N);

    for (int i = 0; i<_N; i++)
    {
        Likelihood val_i = _particles[i].computeLikelihood(measurement, world, _measurmentNoise);
        result.push_back(val_i);
    }     
    return result;
}

Pose ParticleFilterBase::get_average_state()
{
    normaliseWeights();
    // Calculate Weighted Average over samples
    Pose weightedPosition = {0,0,0};
    for (int i = 0; i<_N; i++)
    {
        Pose position_i = _particles[i].getPosition();
        Likelihood weightFactor = _particles[i].getWeight();

        weightedPosition[0] += weightFactor * position_i[0];
        weightedPosition[1] += weightFactor * position_i[1];
        weightedPosition[2] += weightFactor * position_i[2];
    }

    return weightedPosition;
}

void ParticleFilterBase::printAllParticles()
{
    for (int i  = 0 ; i<_N; i++)
    {
        _particles[i].print(i);
    }
}

double ParticleFilterBase::findMaxWeight()
{   
    // Lambda that compares the Weight value of Two particles
    auto compareAttribute = []( Particle &particle1, const Particle &particle2){return particle1.getWeight() < particle2.getWeight();};
    // Find the an iterator of the particle with the highest weight
    auto it = std::max_element(_particles.begin(), _particles.end(),compareAttribute);
    // Return the value of the weight of the found iterator
    return it->getWeight(); 
}

void ParticleFilterBase::normaliseWeights()
{
    // Lambda that returns the cumulative weight given a particle p
    auto accumulateWeight = [](Likelihood i, const Particle& p){return i + p.getWeight();};
    // Compute the cumulative weight of all the particles 
    Likelihood totalSum = std::accumulate(begin(_particles), 
                                          end(_particles),
                                          Likelihood(0.0),
                                          accumulateWeight);
    // lambda that divides the particle p by the totalSum variable
    auto divideWeight = [totalSum](Particle &p){p.setWeight(p.getWeight()/totalSum);};
    // Divide all particles by the totalSum, which thus results in a cumSum of 1.
    std::for_each(_particles.begin(), _particles.end(), divideWeight);    
    return; 
}

int ParticleFilterBase::getNumberParticles()
{
    return _N;
}

PoseList ParticleFilterBase::get_PositionList()
{
    PoseList PositionList;

    for (int i = 0; i<_N; i++)
    {
        PositionList.push_back(_particles[i].getPosition());
    }

    return PositionList;
}
