#pragma once

#include <../include/matplotlibcpp.h>
namespace plt = matplotlibcpp;

#include<vector>
#include<string>

typedef std::vector<double> Pose;

class pltObject
{
public:

pltObject();

pltObject(Pose position, int size, std::string style);

void setPosition(Pose position);

void plot();

void printObject();

Pose getPosition();


private:
Pose _position;
int  _size;
std::map<std::string, std::string> _style;

};
