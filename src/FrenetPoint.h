//
// Created by Stanislav Olekhnovich on 13/10/2017.
// Customized by Anthony Knight on 17/01/2018
//

#ifndef PATH_PLANNING_FRENETPOINT_H
#define PATH_PLANNING_FRENETPOINT_H

// lanes are 4m wide
const double LaneWidthInD = 4.0;

struct FrenetPoint
{
    double S;
    double D;

    FrenetPoint() = default;
    FrenetPoint(double S, double D) : S(S), D(D) {}

	// return d value of centrepoint of desired lane
    inline static double LaneCenterDCoord(int laneNumber) { return LaneWidthInD * laneNumber + LaneWidthInD / 2.0; };

    inline bool IsInLane(int laneNumber) const { return D < LaneWidthInD * (laneNumber + 1)  &&
                D > LaneWidthInD * laneNumber; }

};


#endif //PATH_PLANNING_FRENETPOINT_H
