#pragma once
#include<vector>
#include<string>
#include<iostream>

#include "Object.h"

class LandMark: public Object
{
    public:

    LandMark(Pose position);

    void printObject() const override;
};

typedef std::vector<LandMark> LandMarkList;
