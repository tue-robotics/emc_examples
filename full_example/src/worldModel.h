#ifndef worldModel_H
#define worldModel_H

#include "config.h"

#include <emc/io.h>
#include <emc/data.h>

class WorldModel
{
private:
    double minDistance_;
    
public:
    WorldModel()
    {
        minDistance_ = MAX_RANGE_LRF;
    }
	
    // Method that returns the minimum distance to a wall
    double* getMinimumDistance();

    // Method to determine the minimum distance to a wall
    void setMinimumDistance(emc::LaserData* laser);
};

#endif //worldModel_H
