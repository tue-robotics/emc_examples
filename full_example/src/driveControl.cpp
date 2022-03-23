#include "driveControl.h"
#include "config.h"

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
    
    double distBackward = odomUpdate.x - odom.x;
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
