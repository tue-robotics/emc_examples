#ifndef detection_H
#define detection_H

#include <emc/io.h>
#include <emc/data.h>
#include <emc/bumper.h>

class Detection
{
private:
    emc::IO *inOut;

public:
    Detection(emc::IO *io)
    {
        inOut = io;
        laser = emc::LaserData();
        back_bumper = emc::BumperData();
        back_bumper.contact = false; // initialise to false

        return;
    }
	
    emc::LaserData laser;
    emc::BumperData back_bumper;

    // Method to obtain the sensor data
    bool getSensorData();

    // Method to check if the robot has touched any objects
    bool backBumperTouched();

    // Method to check if any wall is in the neighbourhood of the robot
    bool wallDetected(double minDistance);
};

#endif //detection_H
