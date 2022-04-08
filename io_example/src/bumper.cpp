/**
 * Example code for the MRC (formerly EMC) course.
 * Prints the measurements of the bumper to screen
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


    emc::BumperData frontBumperData;
    emc::BumperData backBumperData;

    std::cout << "Displaying bumper measurements" << std::endl;

    /* Main Execution Loop,
     * this loop keeps running until io.ok returns false, i.e. when a robot error occurs. */
    while(io.ok())
    {
        if(io.readFrontBumperData(frontBumperData)) {
            std::cout  << "front: " << frontBumperData.contact<< std::endl;
        }
        else {
            std::cout  << "front: no data" << std::endl;
        }

        if(io.readBackBumperData(backBumperData)) {
            std::cout  << "back: " << backBumperData.contact<< std::endl;
        }
        else {
            std::cout  << "back: no data" << std::endl;
        }

        // Use this to ensure an execution rate of 20 Hertz
        r.sleep();
    }

    return 0;
}
