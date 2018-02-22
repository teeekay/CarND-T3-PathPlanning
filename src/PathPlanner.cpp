#include "PathPlanner.h"

PathPlanner::PathPlanner(const HighwayMap &map, int startingLane): map(map), targetLane(startingLane)
{
	spdlog::get("console")->info("Loading PathPlanner");
	_PlanL = spdlog::get("Plan");
	_PlanL->info("Loading PathPlanner");
}