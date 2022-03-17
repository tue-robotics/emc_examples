#include "Object.h"

Object::Object()
{
    _position = {0,0,0};
}

Object::Object(const Pose &position) : _position(position){}
 
void Object::setPosition(const Pose &position)
{
    _position = position;
}

void Object::printObject() const
{
    std::cout<<"Pose of Object: "<< _position[0];
    std::cout<<","<< _position[1] << "," <<  _position[2] << std::endl;  
}

Pose Object::getPosition() const
{
    return _position;
}
