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
//	int AddJMTDescriptors(std::vector<FrenetDescriptors> FDPts); //add frenet path calculate Cartesian values
	FrenetPoint GetFrenetSpeed(int PathIndex);
	FrenetDescriptors GetFrenetDescriptorsAt(int PathIndex);
	int TrimPathDequeAtStart(int ElementsToTrim);
	int TrimPathDequeAtEnd(int ElementsToTrim);
	std::vector<CartesianPoint> GetCPath();
	int size();
private:
	std::deque<PathPoint> PathDeque;
	std::shared_ptr<spdlog::logger> _loggerPathTracking;
};

#endif //PATHTRACKER_H