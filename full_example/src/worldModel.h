#include "config.h"

#ifndef worldModel_H
#define worldModel_H

#include <emc/io.h>
#include <emc/data.h>

class WorldModel
        {
private:    
    double minDistance_;
    
public:
WorldModel(){
		minDistance_ = MAX_RANGE_LRF;
	}
	
    double* getMinimumDistance();
    void setMinimumDistance(emc::LaserData* laser); // Method to determine the minimum distance to a wall
};

#endif //worldModel_H	
