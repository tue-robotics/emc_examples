// example.cpp
#include "World.h"
#include "Robot.h"
#include "ParticleFilter.h"
#include "Resampler.h"


int main() {
    // Initialise the Simulation Enviroment
    // ------
    Pose RobotPose = {3,3,1};        //   Robot is initialised in the center of the world.

    Pose lm1 = {2,2,0};
    Pose lm2 = {2,8,0};
    Pose lm3 = {9,2,0};
    Pose lm4 = {8,9,0};
    LandMarkList lms = {lm1,lm2,lm3,lm4};

    int WorldSizeX = 10;
    int WorldSizeY = 10;
    // Initialise the world and LandMarks
    World world (lms,WorldSizeX,WorldSizeY);
    // Robot and measurement Noise Parameters
    // element 1
    // .. todo etc.
    std::vector<double> params = {0.005,0.002,0.2,0.05};
    // Initialise the Robot in the World
    double desiredDist = 0.25;
    double desiredRot  = 0.02;
    Robot rob(RobotPose,params);
    // Determine the length of the simulation 
    int N = 25;
    // ------
    // Initialise the Particle Filter 
    // ------
    int NParticles = 2500;
    ParticleFilter pFilt(world,NParticles);  

    double motion_forward_std= 0.1;
    double motion_turn_std   = 0.2;
    double meas_dist_std     = 0.4;
    double meas_angl_std     = 0.3;
    pFilt.setNoiseLevel(motion_forward_std,motion_turn_std,meas_dist_std,meas_angl_std);
    // ------
    // Loop the simulation for the length of the simulation
    for (int i = 0; i < N; i ++)
    {
        // Plot the state of the World at t = i
        PoseList ParticlePositions = pFilt.get_PositionList();
        world.plotWorld(rob.getPosition(), ParticlePositions,i);
        // The Robot Moves
        rob.move(desiredDist,desiredRot,world);
        // The Robot Performs a Measurement
        measurementList meas = rob.measure(world); 
        // The ParticleFilter incorporates the Measurement 
        pFilt.update(desiredDist,desiredRot,meas,world);
    }
}