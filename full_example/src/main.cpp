#include <emc/io.h>
#include <emc/rate.h>
#include <emc/odom.h>
#include <cmath>

#include "driveControl.h"
#include "detection.h"
#include "worldModel.h"

#include "config.h"

typedef enum {
    drive_forward = 1,
    drive_backward,
    rotate,
} state_t;

int main(int argc, char *argv[])
{
    // Initialization of Robot
    emc::Rate r(EXECUTION_RATE);
    emc::IO io;
    emc::OdometryData odom;
    
    // Initialize the Classes
    DriveControl pico_drive(&io);
    Detection detection(&io);
    WorldModel worldModel;

    // Initialize the State of the State Machine
    state_t state = drive_forward;
    double rotatedAngle = 0.0;
    double distanceBackwards = 0.0;

    /* Main Execution Loop, this loop keeps running until io.ok returns false,
     * hence on a robot error. */
    while(io.ok()) {
    	// Get the Sensor data from the LRF
        if(detection.getSensorData()) {
			// Feed the WorldModel
			worldModel.setMinimumDistance(&(detection.laser));
			
        	// State Machine
			switch(state) {
			// case drive_forward: the robot drives forward until a wall is detected.
			case drive_forward:
				if(detection.wallDetected(*(worldModel.getMinimumDistance()))) {
					// If a wall is detected:
					// stop before we hit the wall 
					pico_drive.stop();
					// reset rotatedAngle to 0 
					rotatedAngle = 0.0;
					// reset distanceBackwards to 0 
					distanceBackwards = 0.0;
					// switch state to move backwards
					state = drive_backward;
				} else {
					pico_drive.driveForward(FORWARD_SPEED);
				}
				break;
			// case drive_backward: the robot drives backward
			case drive_backward:
				// start driving backwards, add distance driven to counter distanceBackwards
				distanceBackwards += pico_drive.driveBackward(FORWARD_SPEED);
				// if we have driven backwards far enough,
				if(fabs(distanceBackwards) >= DIST_BACKWARDS) {
					// we start rotating.
					state = rotate;
				}
				break;
			case rotate:
				// start rotating, add angular displacement to counter rotatedAngle
				rotatedAngle += pico_drive.rotate(ROTATE_SPEED);	
				// if we have rotated enough,
				if(fabs(rotatedAngle) >= 0.5*M_PI) {
					// start driving again
					state = drive_forward;
				}
				break;
				
			default:
				pico_drive.stop();
				break;
			}
        } else {
			 pico_drive.stop();
		}
        // Use this to ensure an execution rate of 20 Hertz.
        r.sleep();
    }
    

    return 0;
}
