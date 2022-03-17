#pragma once

#include<vector>
#include "LandMark.h"

typedef std::vector<Pose> PoseList;

class World
{
    public:
    World(LandMarkList lms, int sz_x ,int sz_y);

    LandMarkList getLandMarks() const;
    LandMark getLandMark(int id) const;

    void plotWorld(Pose robotPose, PoseList particles, Pose AverageParticle, int i);

    int _size_x;
    int _size_y;

    private:  
    LandMarkList _lms;
};
