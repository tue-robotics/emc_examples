#ifndef detection_H
#define detection_H

#include <emc/io.h>
#include <emc/data.h>

class Detection{
private:
    emc::IO *inOut;

public:
 Detection(emc::IO *io){
        inOut = io;
        laser = emc::LaserData();

        return;
    }

    emc::LaserData laser;
    bool getSensorData(); // Method to obtain the sensordata
    bool wallDetected(double minDistance);// Method to check if any wall is in the neighbourhood of the robot
};

#endif //detection_H
