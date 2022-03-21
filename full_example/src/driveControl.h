#ifndef driveControl_H
#define driveControl_H

#include <emc/io.h>
#include <emc/odom.h>

class DriveControl
{
private:
    emc::IO *inOut;
    emc::OdometryData odom; // [x,y,a]

public:
    DriveControl(emc::IO *io)
    {
        inOut = io;
        odom = emc::OdometryData();

        return;
    }

    // Method to go forward with the robot
    void driveForward(double xSpeed);

    // Method to go backward with the robot
    double driveBackward(double xSpeed);

    // Method to rotate with the robot
    double rotate(double aSpeed);

    // Method to stop moving with the robot
    void stop();
};

#endif //driveControl_H
