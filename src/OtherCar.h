//
// Created by Stanislav Olekhnovich on 21/10/2017.
//

#ifndef PATH_PLANNING_OTHERCAR_H
#define PATH_PLANNING_OTHERCAR_H

#include <cmath>

#include "CartesianPoint.h"
#include "FrenetPoint.h"


struct OtherCar
{
	int id;  //aded this in
    CartesianPoint LocationCartesian;
    double XAxisSpeed;
    double YAxisSpeed;
    FrenetPoint LocationFrenet;
    inline double Speed2DMagnitudeMpH() const { return sqrt(XAxisSpeed * XAxisSpeed + YAxisSpeed * YAxisSpeed); } 
	inline double Speed2DMagnitudeMpS() const { return sqrt(XAxisSpeed * XAxisSpeed + YAxisSpeed * YAxisSpeed) * (1609.34 / 3600.0); }//add m/s speed
    inline bool IsInLane(int laneNumber) const { return LocationFrenet.IsInLane(laneNumber); }
};


#endif //PATH_PLANNING_OTHERCAR_H
