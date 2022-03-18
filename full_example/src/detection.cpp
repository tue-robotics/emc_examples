#include "detection.h"
#include "config.h"

    bool Detection::getSensorData() {
        if(inOut->readLaserData(laser)) {
            return true;
        } else {
            return false;
        }
    }

    bool Detection::wallDetected(double minDistance) {
        if(minDistance < MIN_DIST_TO_WALL) {
            return true;
        } else {
            return false;
        }
    }
