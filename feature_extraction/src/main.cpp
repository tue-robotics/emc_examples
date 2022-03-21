/**
 * Example showing a split and merge algorithm for feature extraction.
 * Use pico-teleop to drive around the map, and see it at work.
 * Play around with the variables in the config file, and try to understand what happens.
 */

#include <emc/io.h>
#include <emc/rate.h>

#include "featureExtraction.h"
#include "config.h"

int main(int argc, char *argv[])
{
    // Initialize the robot
    emc::Rate r(EXECUTION_RATE);
    emc::IO io;

    // Create a LaserData object, which will hold the laser data
    // (in "full_example" this is part of the Detection class)
    emc::LaserData laser = emc::LaserData();
    
    // Initialize the feature extraction class
    FeatureExtraction featureExtraction;

    // Loop while we are properly connected
    while(io.ok())
    {
    	// Get the sensor data from the LRF
        if(io.readLaserData(laser))
        {
            // Do feature extraction
            featureExtraction.findLinesAndCorners(&laser);
        } 

        // Use this to ensure an execution rate of 20 Hertz
        r.sleep();
    }

    return 0;
}
