#pragma once
#include<vector>
#include<string>
#include<iostream>

typedef std::vector<double> Pose;

class Object
{
public:

Object();

Object(const Pose &position);

virtual void setPosition(const Pose &position);

virtual Pose getPosition() const;

virtual void printObject() const;




private:
Pose _position;
};
