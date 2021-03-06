#include "AdaptiveParticleFilter.h"

void AdaptiveParticleFilter::update(const double &forwardMotion, const double &angleMotion, const measurementList &measurement, const World &world) 
{
    ParticleList oldParticles = _particles;
    _particles.clear();

    int Nnew = -1;       //Number of New Particles
    int Nreq = _N_max;      //Number of required Particles
    // Internal variables
    BinList binsWithSupport;
    int Nbins = 0;

    while (Nnew < Nreq)
    {
        // Generate a sample 
        int m = _resampler.generateSampleIndex(oldParticles);
        Particle particleM = oldParticles[m];
        // Propagate the sample
        particleM.propagateSample(forwardMotion,
                                  angleMotion,
                                  ParticleFilterBase::_processNoise );
        /// Compute the weight of the generated Sample
        double weightI = particleM.computeLikelihood(measurement,
                                                     world,
                                                     ParticleFilterBase::_measurmentNoise );
        particleM.setWeight(weightI);
        // Add wheighted particle to the new particle set
        _particles.push_back(particleM);
        // Bookkeeping
        Nnew += 1;

        // Map Particle Pose to bin indices
        Pose PoseM   = particleM.getPosition();
        Bin indices = {  floor(PoseM[0] / _resolution[0]),
                         floor(PoseM[1] / _resolution[1]),
                         floor(PoseM[2] / _resolution[2])};

        // Add indices if this bin is empty (i.e. is not in vector yet)
        if(not(idxInBins(indices,binsWithSupport)))
        {
            binsWithSupport.push_back(indices);
            Nbins++;
        }     

        // Update number of required particles (only defined if number of bins with support above 1)
        if(Nbins>1)
        {
            Nreq = computeRequiredParticles(Nbins,_epsilon,_upperQ);
        }
        
        // Constrain required Number of Particles between min and max value
        Nreq = std::max(Nreq,_N_min);
        Nreq = std::min(Nreq,_N_max);
    }
    std::cout<<"Done Resampling: N: "<<Nreq<<" N_max: "<<_N_max<<" N_min: "<<_N_min<<std::endl;
    // Normalize weights
    normaliseWeights();
    // Tell the base that the number of particles has changed
    ParticleFilterBase::resetNumberParticles();
    return;
}

double AdaptiveParticleFilter::computeRequiredParticles(int k, double epsilon, double upperQ)
{
    // For the theoretical background of this formula the reader is reffered to the literature
    // on kld-adaptive particle filters
    
    double x = 1 - 2/(9*(k-1)) + std::sqrt(2/(9*(k-1))) * upperQ;
    return std::ceil((k-1)/(2*epsilon)*x*x*x);
}

bool AdaptiveParticleFilter::idxInBins(Bin &indices, BinList &binsWithSupport)
{
    // This function determines whether a given set indices is already present in the bin list

    // Use the find function to find an element of the vector that is equal to indices
    auto it = std::find(binsWithSupport.begin(),binsWithSupport.end(),indices);

    if (it == binsWithSupport.end())    // If the iterator reaches the end of the vector
    {
        return false;                   // The item is not yet in the list
    }
    else    
    {
        return true;                    // The item is already in the list
    }
}

void AdaptiveParticleFilter::configureAdaptive(std::string resamplingScheme, Likelihood resampleThreshold)
{
    _resamplingScheme = resamplingScheme;
    _resampleThreshold = resampleThreshold;

    _resampler.initaliseResampler(&(_generator),"Adaptive");

    _resolution = {0.2, 0.2, 0.3};
    _epsilon = 0.15;
    _upperQ  = 3;
    _N_min = 50;
    _N_max= 20000;
}
