#include "detection.h"
#include "config.h"

bool Detection::getSensorData() 
{
    bool newdata = false;
    if(inOut->readLaserData(laser)) {
        newdata = true;
    }
    if(inOut->readBackBumperData(back_bumper)) {
        newdata = true;
    }
    return newdata;
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
