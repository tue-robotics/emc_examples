#pragma once
#include<vector>
#include<string>
#include<iostream>

#include "Object.h"

class LandMark: public Object
{
    public:
    // Constructor to create a landmark, given a position
    LandMark(Pose position);
    // Debugging tool, print the landmark
    void printObject() const override;
};

typedef std::vector<LandMark> LandMarkList;
