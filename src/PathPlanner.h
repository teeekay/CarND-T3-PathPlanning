//
// Created by Stanislav Olekhnovich on 16/10/2017.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "HighwayMap.h"
#include "PathPlannerInput.h"
#include "spdlog/spdlog.h"

const int LeftmostLaneNumber = 0;

class PathPlanner
{
public:
	explicit PathPlanner(const HighwayMap &map, int startingLane);
    virtual std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) = 0;
	inline double rad2deg(double x) { return x * 180.0 / M_PI; }
	inline double deg2rad(double x) { return x * M_PI / 180.0; }
	inline double MphToMetersPerSecond(double mphValue) { return mphValue * (1609.34 / 3600.0); }
	double MaxSpeedMpS;
	double MaxSpeedInLaneChangeMpS;
protected:
    const HighwayMap& map;
	int currentLane;  //add this maybe for lane changes
    int targetLane;
	std::shared_ptr<spdlog::logger> _PlanL;
};


#endif //PATH_PLANNING_PATHPLANNER_H
