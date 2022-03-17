#pragma once
#include<vector>
#include<string>
#include<iostream>

typedef std::vector<double> Pose;

class Object
{
    public:
    // Create an empty Object, the position is set to the origin of the coordinate frame
    Object();
    // Create an Object centered at the position of the specified Pose
    Object(const Pose &position);

    // Change the position of the object
    virtual void setPosition(const Pose &position);
    // Retrieve the posistion of the object
    virtual Pose getPosition() const;
    // Debugging tool, print the position of the object
    virtual void printObject() const;
    
    private:
    Pose _position;
};
