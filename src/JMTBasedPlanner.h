
#ifndef PATH_PLANNING_JMTBASEDPLANNER_H
#define PATH_PLANNING_JMTBASEDPLANNER_H

#include "PathPlanner.h"
#include "JMT.h"
#include "RoadMap.h"

#include "spline.h"

const double JMTMaxSpeedMpH = 48.5;// 48.0;//
const double JMTMaxSpeedinLaneChangeMpH = 46.0;// 47.0;//? verify empirically if needed
const double JMTCarinfrontbuffer = 15.0;//20.0

enum EGOState {
	Uninitialized,
	DriveInLane,
	ChangeLane,
	MatchVelocity
};

struct Acc_Jerk
{
	double Max_Vel;
	double Max_Total_Acc;
	double Max_Pos_Acc; //Acceleration
	double Max_Neg_Acc; //braking
	double Max_Fwd_Acc;
	double Max_Lat_Acc;
	double Max_Total_Jerk;
	double Max_Fwd_Jerk;
	double Max_Lat_Jerk;
};

#define KEEP_LANE_MINIMUM_TIME 3000 //3000 mS

class JMTBasedPlanner : public PathPlanner
{
public:
	explicit JMTBasedPlanner(const HighwayMap &map, int startingLane) : PathPlanner(map, startingLane), targetSpeed(.0), EgoState(Uninitialized),
		MaxSpeedMpS(MphToMetersPerSecond(JMTMaxSpeedMpH)), MaxSpeedInLaneChangeMpS(MphToMetersPerSecond(JMTMaxSpeedinLaneChangeMpH)),
		acceleration(0.0), laststep_targetspeed(0.0) {
		
	};
	std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
private:
	double targetSpeed;
	double MaxSpeedInLaneChangeMpS;
	double MaxSpeedMpS;
	double acceleration;
	double laststep_targetspeed;
	double currentSpeedMpS;
	double currentFSpeedMPS; //tracking speed in Frenet Co-ords.
	double MaxVelocityReported;
	EGOState EgoState;
	bool TimerSet;
	std::chrono::system_clock::time_point KeepLaneTimer;

	FrenetPoint LastIterEndpointFPt;

	double safeAcceleration(double accel);
	Acc_Jerk CheckPath(std::vector<CartesianPoint> Path, double time_increment, double v_init=0.0, double a_init=0.0);

//	std::vector<OtherCar> FindCarsAhead(const PathPlannerInput &input);

	char* GetStateName();

	std::vector<CartesianPoint> InitiateTrajectory(RoadMap PlannerMap, PathPlannerInput input);
	std::vector<CartesianPoint> GenerateJMTLaneChangeTrajectory(RoadMap PlannerMap, PathPlannerInput input);
	std::vector<CartesianPoint> GenerateKeepInLaneTrajectory(RoadMap PlannerMap, PathPlannerInput input);

};


#endif //PATH_PLANNING_JMTBASEDPLANNER_H
