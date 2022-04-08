/**
 * Example code for the MRC (formerly EMC) course.
 * Prints the measurements of the odometry to screen
 */

#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <math.h>

#define EXECUTION_RATE 3

int main(int argc, char *argv[])
{
    // Initialize the robot
    emc::Rate r(EXECUTION_RATE);
    emc::IO io;


    emc::OdometryData odomData;
    emc::OdometryData odomUpdate;

    double distanceDriven = 0.0;

    std::cout << "Displaying odometery measurements" << std::endl;

    while(io.ok())
    {
        if(io.readOdometryData(odomData)) {
            break;
        }
        r.sleep();
    }

    /* Main Execution Loop, 
     * this loop keeps running until io.ok returns false, i.e. when a robot error occurs. */
    while(io.ok())
    {
        if(io.readOdometryData(odomUpdate)) {
            std::cout  << "odom x: " << odomUpdate.x
                       << " y: " << odomUpdate.y
                       << " a: " << odomUpdate.a << std::endl;

            double distForward = cos(odomData.a)*(odomUpdate.x - odomData.x) + sin(odomData.a)*(odomUpdate.y - odomData.y) ;
            double delta_time = odomUpdate.timestamp - odomData.timestamp;

            distanceDriven += distForward;
            double velocity = distForward/delta_time;

            std::cout  << "distance driven: " << distanceDriven
                       << " forward velocity: " << velocity << std::endl;

            odomData = odomUpdate;
        }

        // Use this to ensure an execution rate of 20 Hertz
        r.sleep();
    }

    return 0;
}
