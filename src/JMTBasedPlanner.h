//
// JMTBasedPlanner.h
// Created by Anthony M Knight 30/01/2018
//
#ifndef PATH_PLANNING_JMTBASEDPLANNER_H
#define PATH_PLANNING_JMTBASEDPLANNER_H

#include "PathPlanner.h"
#include "RoadMap.h"
#include "Trajectory.h"

#include "spline.h"
#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"



const double JMTCarinfrontbuffer = 20.0;//19.0//20.0  //4 means you've hit it!

enum EGOState {
	Uninitialized,
	DriveInLane,
	ChangeLane
};

#define KEEP_LANE_MINIMUM_TIME 2000 /// in mS

class JMTBasedPlanner : public PathPlanner, public Trajectory
{
public:
	explicit JMTBasedPlanner(const HighwayMap &map, int startingLane):
		PathPlanner(map, startingLane), Trajectory(map), EgoState(Uninitialized),
		DeccelerationLoopCounter(0)
	{
		spdlog::get("console")->info("loading JMTBasedPlanner");
		try
		{
			_JPL = spdlog::get("JPlan");
		}
		catch (const spdlog::spdlog_ex& ex)
		{
			std::cerr << "Log initialization failed: " << ex.what( ) << std::endl;
		}
		_JPL->info("JMTBasedPlanner Startup.");
		_JPL->flush( );
	};
	std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
private:
	double targetSpeed;
	double acceleration;
	double laststep_targetspeed;
	double currentSpeedMpS;
	double currentFSpeedMPS; //tracking speed in Frenet Co-ords.
	double MaxVelocityReported;
	EGOState EgoState;
	bool TimerSet;
	std::chrono::system_clock::time_point KeepLaneTimer;
	int DeccelerationLoopCounter;
	char* GetStateName();
	std::shared_ptr<spdlog::logger> _JPL;
};


#endif //PATH_PLANNING_JMTBASEDPLANNER_H
