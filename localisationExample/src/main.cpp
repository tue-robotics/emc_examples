// example.cpp
#include "World.h"
#include "Robot.h"
#include "ParticleFilter.h"
#include "Resampler.h"

#include <../include/json.hpp>
#include <fstream>


int main() {
    // Load the Configuration file
    std::string filename = "params.json";
    std::ifstream jsonFile(filename);
    nlohmann::json programConfig = nlohmann::json::parse(jsonFile);
    // Initialise the Simulation Enviroment
    // ------
    Pose RobotPose = programConfig["Robot"]["InitialPosition"];        //   Robot is initialised in the center of the world.

    LandMarkList lms;
    for (int i = 0; i < programConfig["World"]["LandMarks"].size(); i++)
    {
        Pose lmi = programConfig["World"]["LandMarks"][i];
        lms.push_back(lmi);
    }

    int WorldSizeX = programConfig["World"]["WorldSize"][0];
    int WorldSizeY = programConfig["World"]["WorldSize"][1];
    // Initialise the world and LandMarks
    World world (lms,WorldSizeX,WorldSizeY);
    // Robot and measurement Noise Parameters
    // element 1
    // .. todo etc.
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
    ParticleFilter pFilt(world,NParticles);  
    //ParticleFilter pFilt(world,mean,sigma,NParticles);  
    auto propagationParameters = programConfig["ParticleFilter"]["PropagationParameters"];
    double motion_forward_std= propagationParameters["motion_forward_std"];
    double motion_turn_std   = propagationParameters["motion_turn_std"];
    double meas_dist_std     = propagationParameters["meas_dist_std"];
    double meas_angl_std     = propagationParameters["meas_angl_std"];

    pFilt.setNoiseLevel(motion_forward_std,motion_turn_std,meas_dist_std,meas_angl_std);

    pFilt.configureResampler( programConfig["ParticleFilter"]["ResamplingAlgorithm"],
                              programConfig["ParticleFilter"]["ResamplingScheme"],
                              programConfig["ParticleFilter"]["ResamplingThreshold"]);
    // ------
    // Loop the simulation for the length of the simulation
    for (int i = 0; i < N; i ++)
    {
        // Plot the state of the World at t = i
        PoseList ParticlePositions = pFilt.get_PositionList();
        Pose AverageParticle = pFilt.get_average_state();
        world.plotWorld(rob.getPosition(), ParticlePositions,AverageParticle,i);
        // The Robot Moves
        rob.move(desiredDist,desiredRot,world);
        // The Robot Performs a Measurement
        measurementList meas = rob.measure(world); 
        // The ParticleFilter incorporates the Measurement 
        pFilt.update(rob._distance_driv,rob._angle_driv,meas,world);
    }
}