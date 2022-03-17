// example.cpp
#include "World.h"
#include "Robot.h"
#include "ParticleFilter.h"
#include "AdaptiveParticleFilter.h"
#include "Resampler.h"

#include <chrono>
#include <../include/json.hpp>
#include <fstream>


int main() {
    // Load the Configuration file
    std::string filename = "params.json";
    std::ifstream jsonFile(filename);
    nlohmann::json programConfig = nlohmann::json::parse(jsonFile);
    // Initialise the Simulation Enviroment
    // ------
    Pose RobotPose = programConfig["Robot"]["InitialPosition"];        
    // Initialise the Landmark Positions
    LandMarkList lms;
    for (int i = 0; i < programConfig["World"]["LandMarks"].size(); i++)
    {
        Pose lmi = programConfig["World"]["LandMarks"][i];
        lms.push_back(lmi);
    }
    // Initialise the world size
    int WorldSizeX = programConfig["World"]["WorldSize"][0];
    int WorldSizeY = programConfig["World"]["WorldSize"][1];
    // Initialise the world and LandMarks
    World world (lms,WorldSizeX,WorldSizeY);
    // Robot and measurement Noise Parameters
    auto NoiseParameters = programConfig["Robot"]["NoiseParameters"];
    std::vector<double> params = {  NoiseParameters["_std_forward"],
                                    NoiseParameters["_std_turn"],
                                    NoiseParameters["_std_meas_dist"],
                                    NoiseParameters["_std_meas_angl"]      };
    // Initialise the Robot in the World
    double desiredDist = programConfig["Control"]["DesiredForwardVelocity"];
    double desiredRot  = programConfig["Control"]["DesiredAngleVelocity"];
    Robot rob(RobotPose,params);
    // Determine the length of the simulation 
    int N = 25;
    // ------
    // Initialise the Particle Filter 
    // ------
    int NParticles = programConfig["ParticleFilter"]["Particles"];
    // Extract the chosen particleFilter algorithm from the JSON-file
    std::string PFFlavor = programConfig["ParticleFilter"]["ParticleFilterFlavor"];
    bool ConventionalPF  = PFFlavor.compare("Conventional") == 0;
    // Choose the particleFilter Algorithm to be used 
    ParticleFilterBase* pFilt;
    if (ConventionalPF) // Conventional PF
    {
        pFilt = new ParticleFilter(world,NParticles);
        //pFilt = new ParticleFilter(world,mean,sigma,NParticles);  
        pFilt -> configureResampler( programConfig["ParticleFilter"]["ResamplingAlgorithm"],
                                     programConfig["ParticleFilter"]["ResamplingScheme"],
                                     programConfig["ParticleFilter"]["ResamplingThreshold"]);
    }
    else // Adaptive PF
    {
        pFilt = new AdaptiveParticleFilter(world,NParticles);        
        pFilt -> configureAdaptive(programConfig["ParticleFilter"]["ResamplingScheme"],
                                   programConfig["ParticleFilter"]["ResamplingThreshold"]);
    }

    auto propagationParameters = programConfig["ParticleFilter"]["PropagationParameters"];
    double motion_forward_std= propagationParameters["motion_forward_std"];
    double motion_turn_std   = propagationParameters["motion_turn_std"];
    double meas_dist_std     = propagationParameters["meas_dist_std"];
    double meas_angl_std     = propagationParameters["meas_angl_std"];

    pFilt->setNoiseLevel(motion_forward_std,motion_turn_std,meas_dist_std,meas_angl_std);
    // ------
    // Loop the simulation for the length of the simulation
    for (int i = 0; i < N; i ++)
    {
        // Plot the state of the World at t = i
        PoseList ParticlePositions = pFilt->get_PositionList();
        Pose AverageParticle = pFilt->get_average_state();
        world.plotWorld(rob.getPosition(), ParticlePositions, AverageParticle,i);
        // Start Clock of this iteration (we dont include time taken by viz)
        auto start = std::chrono::steady_clock::now();
        // The Robot Moves
        rob.move(desiredDist,desiredRot,world);
        // The Robot Performs a Measurement
        measurementList meas = rob.measure(world); 
        // The ParticleFilter incorporates the Measurement 
        pFilt->update(rob._distance_driv, rob._angle_driv, meas, world);
        // Stop Clock of this iteration (we dont include time taken by viz)
        auto end = std::chrono::steady_clock::now();

        std::cout<<"Timestep "<<i<<" Complete. Time Taken: "<<
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0 << " milliseconds"<<"\n";
    }
}
