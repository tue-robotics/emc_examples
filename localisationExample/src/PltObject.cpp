#include<vector>
#include<string>
#include<iostream>

#include "PltObject.h"

pltObject::pltObject(Pose pos, int sz, std::string style) : _position(pos),_size(sz)
{
    std::string key = "marker";
    _style.insert(std::make_pair(key, style));
}

void pltObject::setPosition(Pose position)
{
    _position = position;
}

void pltObject::plot()
{
    std::vector<double> X {_position[0]};
    std::vector<double> Y {_position[1]};
    plt::scatter(X,Y,5.0,_style);

    plt::save("./basic.png");
}

void pltObject::printObject()
{
    std::cout<<"Pose of Object: "<< _position[0];
    std::cout<<","<< _position[1] << "," <<  _position[2] << std::endl;  
}

Pose pltObject::getPosition()
{
    return _position;
}