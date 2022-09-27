#include "World.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

    World::World(LandMarkList lms ,int sz_x,int sz_y): _lms(lms), _size_x(sz_x), _size_y(sz_y) {};

    LandMarkList World::getLandMarks() const
    {
        return _lms;
    }

    LandMark World::getLandMark(int id) const
    {
        return _lms[id];
    }

    void World::plotWorld(Pose robotPose, PoseList particles, Pose AverageParticle, int i = 0)
    {
        if (i==0)
        {
            plt::figure_size(500, 500);
        }
        else
        {
            plt::clf();
        }

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
        plt::scatter(X,Y,100,{{"marker","s"},{"color","black"}});

        // Plot Particle Positions
        std::vector<double> Xparticles;
        std::vector<double> Yparticles;

        for (int i = 0; i< particles.size(); i++)
        {
            Xparticles.push_back(particles[i][0]);
            Yparticles.push_back(particles[i][1]);
        }

        plt::scatter(Xparticles, Yparticles,5,{{"color","orange"}});

        // Plot Robot Position
        std::vector<double> xRobot = {robotPose[0]};
        std::vector<double> yRobot = {robotPose[1]};        
        std::vector<double> thRobot= {robotPose[2]};
        plt::scatter(xRobot, yRobot,100,{{"color","darkgreen"}});
        // Plot line denoting robot true orientation
        double orientationLineLength = 0.3;
        std::vector<double> orLineX = {robotPose[0], robotPose[0] + orientationLineLength*std::cos(robotPose[2])};
        std::vector<double> orLineY = {robotPose[1], robotPose[1] + orientationLineLength*std::sin(robotPose[2])};
        plt::plot(orLineX,orLineY,{{"color","darkgreen"}});

        // Plot Average Particle Position
        std::vector<double> xAverage = {AverageParticle[0]};
        std::vector<double> yAverage = {AverageParticle[1]};        
        plt::scatter(xAverage, yAverage,100,{{"color","red"}});
        // Plot line denoting robot average orientation
        orLineX = {AverageParticle[0], AverageParticle[0] + orientationLineLength*std::cos(AverageParticle[2])};
        orLineY = {AverageParticle[1], AverageParticle[1] + orientationLineLength*std::sin(AverageParticle[2])};
        plt::plot(orLineX,orLineY,{{"color","red"}});


        plt::xlim(0, _size_x);
        plt::ylim(0, _size_y);
        plt::pause(0.1);
        plt::save("StateOfTheWorld@t=" + std::to_string(i) + ".png");
        return;
    }
