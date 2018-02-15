
#ifndef PATH_PLANNING_JMTBASEDPLANNER_H
#define PATH_PLANNING_JMTBASEDPLANNER_H

#include "PathPlanner.h"
#include "RoadMap.h"
#include "Trajectory.h"

#include "spline.h"


const double JMTCarinfrontbuffer = 10.0;//15.0//20.0

enum EGOState {
	Uninitialized,
	DriveInLane,
	ChangeLane,
	MatchVelocity
};

#define KEEP_LANE_MINIMUM_TIME 2000 //3000 mS

class JMTBasedPlanner : public PathPlanner, public Trajectory
{
public:
	explicit JMTBasedPlanner(const HighwayMap &map, int startingLane):
		PathPlanner(map, startingLane), Trajectory(map), EgoState(Uninitialized) { };
	std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
private:
	double targetSpeed;
//	double MaxSpeedInLaneChangeMpS;
//	double MaxSpeedMpS;
	double acceleration;
	double laststep_targetspeed;
	double currentSpeedMpS;
	double currentFSpeedMPS; //tracking speed in Frenet Co-ords.
	double MaxVelocityReported;
	EGOState EgoState;
	bool TimerSet;
	std::chrono::system_clock::time_point KeepLaneTimer;

//	FrenetPoint LastIterEndpointFPt;

	char* GetStateName();
//	Trajectory Traj;
};


#endif //PATH_PLANNING_JMTBASEDPLANNER_H
