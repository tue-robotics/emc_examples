#include "ParticleFilterBase.h"

ParticleFilterBase::ParticleFilterBase(World world, int N)
{
    _N = N;
    double long weight = 1.0/double(N);  
    for (int i = 0; i<_N; i++)
    {
        Particle particle_i(world, weight, &_generator);
        _particles.push_back(particle_i);
    }
}


ParticleFilterBase::ParticleFilterBase(World world, double mean[3], double sigma[3], int N)
{
    _N = N;
    double long weight = 1/_N;    
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
        double long val_i = _particles[i].computeLikelihood(measurement,world, _measurmentNoise);
        result.push_back(val_i);
    }     
    return result;
}

Pose ParticleFilterBase::get_average_state()
{
    // Calculate sum of Weights of all particles
    double long sum_weight = 0;
    for (int i = 0; i<_N; i++)
    {
        sum_weight += _particles[i].getWeight();
    }

    // Calculate Weighted Average over samples
    Pose weightedPosition = {0,0,0};
    for (int i = 0; i<_N; i++)
    {
        Pose position_i       = _particles[i].getPosition();
        double long weightFactor   = _particles[i].getWeight()/sum_weight;

        weightedPosition[0]  +=  weightFactor * position_i[0];
        weightedPosition[1]  +=  weightFactor * position_i[1];
        weightedPosition[2]  +=  weightFactor * position_i[2];
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
    double long maxWeight = 0;
    for (int i = 0; i<_N; i++)
    {
        double long weighti = _particles[i].getWeight();

        if (weighti > maxWeight)
        {
            maxWeight = weighti;
        }
    }

    return maxWeight;
}

void ParticleFilterBase::normaliseWeights()
{
    double long totalSum = 0;

    for (int i = 0; i<_N; i++)
    {
        totalSum += _particles[i].getWeight();
    }
    
    for (int i = 0; i<_N; i++)
    {
        double long newWeighti = _particles[i].getWeight()/totalSum;
        _particles[i].setWeight(newWeighti);
    }
    
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

void ParticleFilterBase::plotAllParticles()
{
    std::vector<double> Xparticles;
    std::vector<double> Yparticles;

    for (int i = 0; i< _particles.size(); i++)
    {
        Xparticles.push_back(_particles[i].getPosition()[0]);
        Yparticles.push_back(_particles[i].getPosition()[1]);
    }

    plt::scatter(Xparticles,Yparticles,10);

    Pose averagePose = get_average_state();
    std::vector<double> averageX;
    std::vector<double> averageY;

    plt::scatter(averageX,averageY,150);
}