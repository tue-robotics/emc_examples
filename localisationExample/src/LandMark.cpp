#include "LandMark.h" 

LandMark::LandMark(Pose position) : Object(position) {}

void LandMark::printObject() const
{
    std::string type = "Landmark";
    std::cout<< "Object: " << type << std::endl;
    Object::printObject();
}
