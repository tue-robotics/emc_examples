#include "World.h"

    World::World(LandMarkList lms ,int sz_x,int sz_y): _lms(lms), _size_x(sz_x), _size_y(sz_y) {};

    LandMarkList World::getLandMarks()
    {
        return _lms;
    }

    LandMark World::getLandMark(int id)
    {
        return _lms[id];
    }

    void World::plotWorld(Pose robotPose, PoseList particles, Pose AverageParticle, int i = 0)
    {
        plt::figure_size(1000,1000);


        // Plot Landmark Positions
        std::vector<double> X;
        std::vector<double> Y;

        X.reserve(_lms.size());
        Y.reserve(_lms.size());

        for (int i = 0; i < _lms.size(); i++)
        {
            Pose posi = _lms[i].getPosition();
            X.push_back(posi[0]);
            Y.push_back(posi[1]);            
        }
        plt::scatter(X,Y,100); // Todo get size and style

        // Plot Particle Positions
        std::vector<double> Xparticles;
        std::vector<double> Yparticles;

        for (int i = 0; i< particles.size(); i++)
        {
            Xparticles.push_back(particles[i][0]);
            Yparticles.push_back(particles[i][1]);
        }

        plt::scatter(Xparticles,Yparticles,10);

        // Plot Robot Position
        std::vector<double> xRobot = {robotPose[0]};
        std::vector<double> yRobot = {robotPose[1]};        
        plt::scatter(xRobot,yRobot,100);

        // Plot Average Particle Position
        std::vector<double> xAverage = {AverageParticle[0]};
        std::vector<double> yAverage = {AverageParticle[1]};        
        plt::scatter(xAverage,yAverage,100);

        

        plt::xlim(0,_size_x);
        plt::ylim(0,_size_y);
        plt::save("StateOfTheWorld@t=" + std::to_string(i) + ".png");
         plt::close();
        return;
    }

