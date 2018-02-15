#include "PathPlanner.h"

PathPlanner::PathPlanner(const HighwayMap &map, int startingLane): map(map), targetLane(startingLane)
{
	this->_logger = spdlog::get("PathPlannerLogger");
}