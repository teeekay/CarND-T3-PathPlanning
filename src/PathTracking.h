//
// PathTracking.h
// Created by Anthony M Knight 25/01/2018
//
#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include <deque>

#include "PathPlanner.h"
#include "spline.h"
#include "spdlog/spdlog.h"

struct PathPoint
{
	CartesianPoint CPt;
	CartesianPoint CVPt; //Cartesian Velocity
	CartesianPoint CAPt; //Cartesian Acceleration
	FrenetPoint FPt;
	FrenetPoint FVPt;  //Velocity in Frenet
	FrenetPoint FAPt;  //Acceleration in Frenet
};



class PathTracking
{
public:
	PathTracking();
	int AddPathPoint(PathPoint PPt); //add point returns size of deque
	FrenetPoint GetFrenetSpeed(int PathIndex);
	int TrimPathDequeAtStart(int ElementsToTrim);
	int TrimPathDequeAtEnd(int ElementsToTrim);
private:
	std::deque<PathPoint> PathDeque;
	std::shared_ptr<spdlog::logger> _loggerPathTracking;
};

#endif //PATHTRACKER_H