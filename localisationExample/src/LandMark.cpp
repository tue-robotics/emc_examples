#include<vector>
#include<string>
#include<iostream>

#include "LandMark.h" 

LandMark::LandMark(Pose position) : pltObject(position,6,"s") {}

void LandMark::plot()
{
    pltObject::plot();
}

void LandMark::printObject()
{
    std::string type = "Landmark";
    std::cout<< "Object: " << type << std::endl;
    pltObject::printObject();
}

Pose LandMark::getPosition()
{
    return pltObject::getPosition();
}