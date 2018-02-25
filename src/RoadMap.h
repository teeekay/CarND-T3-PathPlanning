//
// RoadMap.h
// Created by Anthony M Knight 25/01/2018
//

#ifndef ROADMAPLANNER_H
#define ROADMAPLANNER_H

#include "PathPlanner.h"
#include "spline.h"
#include "spdlog/spdlog.h"

#define MAX_S 6945.554


class RoadMap
{
public:
	RoadMap(const HighwayMap &map, PathPlannerInput input) :
		map(map), ThisStepInput(input), EgoCar(input.LocationFrenet), PlanTargetLane(input.LocationFrenet.GetLane( ))
	    { _RML = spdlog::get("Pred"); };
	int CreateRoadMap();
	int check_lanes();
	int CheckForSlowCarsAhead();
	int CheckForSlowCarsAhead(double distance);
	int CheckForSlowCarInOtherLane(int targetLane, double distance);
	int CheckForLaneChange();
	int SetTarget();
	int PlanTargetLane;
private:
// vectors storing positions of cars and clearances between the EgoCar and other car over time
	std::vector<std::vector<std::vector< std::vector <int>>>> RoadMapDeck;
	//Layers in RoadMapDeck split between Ego Car and OtherCars 
	const int EgoCarLayer = 0;
	const int OtherCarsLayer = 1;

	std::vector<int> EgoInTime; //vector of Egopositions as time progresses
	std::vector<std::vector<int>> clearlanelengthsFWD;// (NumIncrements, std::vector<int>(NumLanes));
	std::vector<std::vector<int>> clearlanelengthsBACK;// (NumLanes, std::vector<int>(NumLanes));
	std::vector<int> MinLaneClearFWD; // over entire projection
	std::vector<int> MinLaneClearBACK; // over entire projection

	PathPlannerInput ThisStepInput;
	const HighwayMap& map;

	FrenetPoint EgoCar;
	double EgoSpeedMpS;

	//spdlog pointer
	std::shared_ptr<spdlog::logger> _RML;

	//	std::vector< std::vector <int>> RoadMapAStarCost;
	//int GoalLane;  //destination target in A* planner
	//int GoalHorizon;  //destination target in A* planner
					  

};

#endif //ROADMAPLANNER_H
