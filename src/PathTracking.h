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
	FrenetDescriptors FDPt;
	PathPoint() = default;
};



class PathTracking
{
public:
	PathTracking();

	int AddPathPoint(PathPoint PPt); //add point returns size of deque

	int TrimPathDequeAtStart(int ElementsToTrim);
	int TrimPathDequeAtEnd(int ElementsToTrim);

	FrenetDescriptors GetFrenetDescriptorsAt(int PathIndex);
	FrenetPoint GetFrenetSpeed(int PathIndex);

	std::vector<CartesianPoint> GetCPath();

	int size();

	int logpath( );
private:
	std::deque<PathPoint> PathDeque;
	std::shared_ptr<spdlog::logger> _PTL;
};

#endif //PATHTRACKER_H