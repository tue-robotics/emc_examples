#include "PltObject.h"

pltObject::pltObject()
{
    std::string key = "marker";
    _style.insert(std::make_pair(key, ""));

    _position = {0,0,0};

    _size = 5;
}

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

    std::cout<< _position[0] <<std::endl;

    plt::scatter(X,Y);//,_style);
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