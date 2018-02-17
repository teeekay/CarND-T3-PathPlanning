#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "PathPlanner.h"
#include "RoadMap.h"
#include "JMT.h"
#include "PathTracking.h"
#include "spdlog/spdlog.h"

#include "spline.h"

#define G_FORCE_MPS 9.8

const double JMTMaxSpeedMpH = 49.0;// 48.0;//
const double JMTMaxSpeedinLaneChangeMpH = 46.0;//

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


class Trajectory
{
public:
	explicit Trajectory(const HighwayMap &map): map2(map), 
		MaxSpeedMpS(MphToMetersPerSecond(JMTMaxSpeedMpH)), 
		MaxSpeedInLaneChangeMpS(MphToMetersPerSecond(JMTMaxSpeedinLaneChangeMpH))
	    { _logger2 = spdlog::get("PathPlannerLogger"); }
	
	std::vector<CartesianPoint> InitiateTrajectory(PathPlannerInput input);
	std::vector<CartesianPoint> GenerateJMTLaneChangeTrajectory(PathPlannerInput input, int TargetLane, double DesiredVelocity, bool truncate=false);
	std::vector<CartesianPoint> OldGenerateJMTLaneChangeTrajectory(PathPlannerInput input, int TargetLane, double DesiredVelocity, bool truncate = false);
	std::vector<CartesianPoint> GenerateKeepInLaneTrajectory(PathPlannerInput input, double DesiredVelocity , bool truncate=false);
	std::vector<CartesianPoint> OldGenerateKeepInLaneTrajectory(PathPlannerInput input, double DesiredVelocity, bool truncate = false);
	double GetAcceleration();
	void LogFPath(std::vector<FrenetPoint> FPath);
	void LogCPath(std::vector<CartesianPoint> CPath);
	const double MaxSpeedInLaneChangeMpS;
	const double MaxSpeedMpS;
private:

	std::vector<CartesianPoint> TrimPath(std::vector<CartesianPoint> &Path, int NumberPoints);
	std::vector<FrenetPoint> GenerateJMTPath(FrenetPoint LastFPt, FrenetPoint DestFPt, FrenetPoint LastSpeed, FrenetPoint TargetSpeed, double T,
		                             bool GenerateZero = false, FrenetPoint LastAccel = { 0.0,0.0 }, FrenetPoint TargetAccel = { 0.0,0.0 });
	std::vector<FrenetDescriptors> GenerateJMTDPath(FrenetPoint LastFPt, FrenetPoint DestFPt, FrenetPoint LastSpeed, FrenetPoint TargetSpeed, double T,
		                             bool GenerateZero = false, FrenetPoint LastAccel = { 0.0,0.0 }, FrenetPoint TargetAccel = { 0.0,0.0 });

	void OffsetPath(std::vector<CartesianPoint> &CPath, CartesianPoint LastCPt);
	CartesianPoint GetOffset(CartesianPoint CalcCPt, CartesianPoint KnownCPt);
	FrenetPoint GetFinalFAccel(std::vector<CartesianPoint> const & CPath);
	
	inline double MphToMetersPerSecond(double mphValue) { return mphValue * (1609.34 / 3600.0); }

	const HighwayMap& map2;
	const double MaxFwdAccelerationMpSsq = 0.25 * G_FORCE_MPS; /* 0.25 times force of gravity */
	const double MaxBrakingAccelerationMpSsq = - 0.35 * G_FORCE_MPS; /* -0.25 times force of gravity */
	const double SimulatorRunloopPeriod = 0.02;

	PathTracking BuiltPath;

//	double TargetSpeed;
	double Acceleration;
//	double EndSpeed;
//	double currentSpeedMpS;
	double MaxVelocityReported;
	FrenetPoint LastIterEndpointFPt;
	double GetSafeAcceleration(double accel);
	Acc_Jerk CheckPath(std::vector<CartesianPoint> Path, double time_increment, double v_init = 0.0, double a_init = 0.0);
	std::shared_ptr<spdlog::logger> _logger2;
};


#endif //TRAJECTORY_H