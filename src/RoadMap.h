//
// RoadMap.h
// Created by Anthony M Knight 25/01/2018
//

#ifndef ROADMAPLANNER_H
#define ROADMAPLANNER_H

#include "PathPlanner.h"
#include "spline.h"
#include "spdlog/spdlog.h"




class RoadMap
{
public:
	RoadMap(PathPlannerInput input) :
		ThisStepInput(input), EgoCar(input.LocationFrenet), PlanTargetLane(input.LocationFrenet.GetLane()) 
	    { _loggerRoadMap = spdlog::get("PathPlannerLogger"); };
	int CreateRoadMap();
	int check_lanes();
	int CheckForSlowCarsAhead();
	int CheckForSlowCarsAhead(double distance);
	int CheckForLaneChange();
	int SetTarget();
	int PlanTargetLane;
private:
	PathPlannerInput ThisStepInput;  //
	std::vector<std::vector<std::vector< std::vector <int>>>> RoadMapDeck;
	std::vector<std::vector<int>> clearlanelengthsFWD;// (NumIncrements, std::vector<int>(NumLanes));
	std::vector<std::vector<int>> clearlanelengthsBACK;// (NumLanes, std::vector<int>(NumLanes));
	std::vector<int> MinLaneClearFWD;// (NumLanes, 0);
	std::vector<int> MinLaneClearBACK;
	std::vector< std::vector <int>> RoadMapAStarCost;
	FrenetPoint EgoCar;
	double EgoSpeedMpS; //estimate of speed of car.
	int GoalLane;  //destination target in A* planner
	int GoalHorizon;  //destination target in A* planner
					  //Layers in RoadMapDeck split between Ego Car and OtherCars 
	const int EgoCarLayer = 0;
	const int OtherCarsLayer = 1;
	std::shared_ptr<spdlog::logger> _loggerRoadMap;
};

#endif //ROADMAPLANNER_H
