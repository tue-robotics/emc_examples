#include "driveControl.h"
#include "config.h"

    void DriveControl::driveForward(double Xspeed) {
        inOut->readOdometryData(odom);
        inOut->sendBaseReference(Xspeed, 0.0, 0.0);
    }

    double DriveControl::driveBackward(double Xspeed) {
        emc::OdometryData odomUpdate;
        inOut->readOdometryData(odomUpdate);

        inOut->sendBaseReference(-Xspeed, 0.0, 0.0);

        double distBackward = odomUpdate.x - odom.x;
        odom = odomUpdate;

        return distBackward;
    }

    double DriveControl::rotate(double Aspeed) {
        emc::OdometryData odomUpdate;
        inOut->readOdometryData(odomUpdate);

        inOut->sendBaseReference(0.0, 0.0, Aspeed);

        double rotationRelative = odomUpdate.a - odom.a;
        odom = odomUpdate;

        return rotationRelative;
    }

    void DriveControl::stop() {
        driveForward(0.0);
    }
