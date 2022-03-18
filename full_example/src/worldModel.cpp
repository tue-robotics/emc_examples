#include "worldModel.h"

double* WorldModel::getMinimumDistance()
{
    return &minDistance_;
}

void WorldModel::setMinimumDistance(emc::LaserData* laser)
{
     double aux = laser->ranges[0];
     for(int i = 1; i < laser->ranges.size(); ++i) {
            if(laser->ranges[i] < aux) {
                aux = laser->ranges[i];
            }
        }
      minDistance_ = aux;
}
