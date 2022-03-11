#pragma once

#include<vector>
#include "PltObject.h"

class LandMark: public pltObject
{

public:

LandMark(Pose position);

void plot();
void printObject();

Pose getPosition();


};

typedef std::vector<LandMark> LandMarkList;
