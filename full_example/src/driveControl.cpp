#include "driveControl.h"
#include "config.h"

#include <math.h>

void DriveControl::driveForward(double xSpeed)
{
    inOut->readOdometryData(odom);
    inOut->sendBaseReference(xSpeed, 0.0, 0.0);
}

double DriveControl::driveBackward(double xSpeed)
{
    emc::OdometryData odomUpdate;
    inOut->readOdometryData(odomUpdate);
    
    inOut->sendBaseReference(-xSpeed, 0.0, 0.0);
    
    // coordinate transformation to get distance in the negative x direction relative to the robot.
    double distBackward = - (cos(odom.a)*(odomUpdate.x - odom.x)
                           + sin(odom.a)*(odomUpdate.y - odom.y));
    odom = odomUpdate;
    
    return distBackward;
}
    
double DriveControl::rotate(double aSpeed)
{
    emc::OdometryData odomUpdate;
    inOut->readOdometryData(odomUpdate);

    inOut->sendBaseReference(0.0, 0.0, aSpeed);
    
    double rotationRelative = odomUpdate.a - odom.a;
    odom = odomUpdate;

    return rotationRelative;
}

void DriveControl::stop()
{
    driveForward(0.0);
}
