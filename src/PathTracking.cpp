
#include <vector>
#include <deque>
#include <chrono>
#include <iostream>
#include <iomanip> //for printing roadmap2.
#include <algorithm>
#include <iterator>

#include "PathTracking.h"

PathTracking::PathTracking()
{
	_loggerPathTracking = spdlog::get("PathPlannerLogger");
	_loggerPathTracking->info("PathTracking Startup");
	
//#ifdef DEBUG
	PathPoint test;
	test.CPt = { 1.0, 10.0, 3.0 };
	test.FDPt.Displacement = { 100.0, 2.0 };
	test.FDPt.Velocity = { 3.0, 0.1 };
	for (int i = 0; i < 10; i++)
	{
		test.FDPt.Velocity.D += double(i);
		int els = AddPathPoint(test);
		_loggerPathTracking->info("PathTracking: Now {} els in Path", els);
	}
	for (int i = 0; i < 10; i++){
		FrenetPoint FVPt = GetFrenetSpeed(i);
		_loggerPathTracking->info("PathTracking: test {}: Frenet Velocity is [ {:3.2f}, {:3.2f} ]",
			i, FVPt.S, FVPt.D);
	}
	int size = TrimPathDequeAtStart(3);
	FrenetPoint FVPt = GetFrenetSpeed(0);
	_loggerPathTracking->info("PathTracking: new start after trimming 3: Frenet Velocity is [ {:3.2f}, {:3.2f} ]",
		FVPt.S, FVPt.D);
	size = TrimPathDequeAtEnd(3);
	FVPt = GetFrenetSpeed(size-1);
	_loggerPathTracking->info("PathTracking: new end after trimming 3 -new size is {}: Frenet Velocity is [ {:3.2f}, {:3.2f} ]",
		size, FVPt.S, FVPt.D);
	_loggerPathTracking->flush();
	PathDeque.clear();
//#endif //DEBUG
	
}

int PathTracking::AddPathPoint(PathPoint PPt)
{
	PathDeque.push_back(PPt);
	return PathDeque.size();
}

int PathTracking::size()
{
	return PathDeque.size();
}

FrenetDescriptors PathTracking::GetFrenetDescriptorsAt(int PathIndex)
{
	return PathDeque.at(PathIndex).FDPt;
}



FrenetPoint PathTracking::GetFrenetSpeed(int PathIndex)
{
	return PathDeque.at(PathIndex).FDPt.Velocity;
}


std::vector<CartesianPoint> PathTracking::GetCPath()
{
	std::vector<CartesianPoint> CPath;
	for (PathPoint lPPt : PathDeque)
	{
		CPath.push_back(lPPt.CPt);
	}
	return(CPath);
}


// Remove points at start of Deque which we have already passed over.
// Maybe leave 1 additional point which would match curent location
// Returns size of new PathDeque
int PathTracking::TrimPathDequeAtStart(int ElementsToTrim)
{
	PathDeque.erase(PathDeque.begin(), PathDeque.begin() + (ElementsToTrim));
	return PathDeque.size();
}

// Remove points at end of Deque when Built Path is truncated
// Returns size of new PathDeque
int PathTracking::TrimPathDequeAtEnd(int ElementsToTrim)
{

	PathDeque.erase(PathDeque.end() - (ElementsToTrim), PathDeque.end());
	return PathDeque.size();
}