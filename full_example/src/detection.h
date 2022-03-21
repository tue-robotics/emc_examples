#ifndef detection_H
#define detection_H

#include <emc/io.h>
#include <emc/data.h>

class Detection
{
private:
    emc::IO *inOut;

public:
    Detection(emc::IO *io)
    {
        inOut = io;
        laser = emc::LaserData();

        return;
    }
	
    emc::LaserData laser;

    // Method to obtain the sensor data
    bool getSensorData();

    // Method to check if any wall is in the neighbourhood of the robot
    bool wallDetected(double minDistance);
};

#endif //detection_H
