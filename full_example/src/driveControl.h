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
    DriveControl(emc::IO *io){
        inOut = io;
        odom = emc::OdometryData();

        return;
    }

    void driveForward(double Xspeed); // Method to go forward with the robot
    double driveBackward(double Xspeed); // Method to go backward with the robot
    double rotate(double Aspeed); // Method to rotate with the robot
    void stop(); // Method to stop moving with the robot
};

#endif //driveControl_H
