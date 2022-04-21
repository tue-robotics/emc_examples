#include "detection.h"
#include "config.h"

bool Detection::getLaserData()
{
    return inOut->readLaserData(laser);
}

bool Detection::getBackBumperData()
{
    return inOut->readBackBumperData(back_bumper);
}

bool Detection::backBumperTouched()
{
    return back_bumper.contact;
}

bool Detection::wallDetected(double minDistance)
{
    if(minDistance < MIN_DIST_TO_WALL) {
        return true;
    } else {
        return false;
    }
}
