//
// JMTBasedPlanner.cpp
// Created by Anthony M Knight 30/01/2018
//

#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip> //for printing roadmap.
#include <algorithm>
#include <iterator>

#include "JMTBasedPlanner.h"
#include "spdlog/spdlog.h"

//#define DEBUG


// if we plan 50 points ahead at maximum speed of 49.5 mph = 22.1 mps, and at 0.02 sec intervals, we will only go out 22.1 m.


static const int XAxisPlanningHorizon = 60;
const double CriticalThresholdInMeters = 20; //15
const double SimulatorRunloopPeriod = 0.02;
static const int MaxNumberOfPointsInPath = 120;


char* JMTBasedPlanner::GetStateName(void)
{
	switch (EgoState)
	{
	case Uninitialized:
		return (char *)"Uninitialized";
	case DriveInLane:
		return (char *)"DriveInLane";
	case ChangeLane:
		return (char *)"ChangeLane";
	case MatchVelocity:
		return (char *)"MatchVelocity";
	}
	return (char *)"Invalid - check GetStateName function";
}

//JMTBasedPlanner(const HighwayMap &map, int startingLane) :
//	PathPlanner(map, startingLane),
//	EgoState(Uninitialized)
//{
//	Traj = Trajectory(map);
//};


std::vector<CartesianPoint> JMTBasedPlanner::GeneratePath(PathPlannerInput input)
{
	
	//get a sorted list of cars in this lane in front of car that will be within 30 m.
	std::vector <CartesianPoint> OutputPath;
	std::vector <CartesianPoint> OldPath = input.Path;
	if (input.SpeedMpS > MaxVelocityReported)
	{
		MaxVelocityReported = input.SpeedMpS;
		if (MaxVelocityReported > Trajectory::MaxSpeedMpS)
		{
			_logger->warn("Maximum Velocity of {:3.3f} mps or {:3.3f} mph recorded at ({:3.3f},{:3.3f}).", MaxVelocityReported, input.SpeedMpH, input.LocationCartesian.X, input.LocationCartesian.Y);
			spdlog::get("console")->warn("Maximum Velocity of {:3.3f} mps or {:3.3f} mph recorded at ({:3.3f},{:3.3f}).", MaxVelocityReported, input.SpeedMpH, input.LocationCartesian.X, input.LocationCartesian.Y);
		}
	}
	int pathsize = OldPath.size();// > 5) ? 5 : input.Path.size();
	currentLane = input.LocationFrenet.GetLane();
	currentSpeedMpS = input.SpeedMpS;

	if (pathsize > 2) {
		FrenetPoint CurrentFPt = map.CartesianToFrenet(input.LocationCartesian);
		FrenetPoint PathStartFPt = map.CartesianToFrenet(input.Path.at(0));
		currentFSpeedMPS = 50.0 * sqrt(pow(PathStartFPt.S - CurrentFPt.S, 2.0) + pow(PathStartFPt.D - CurrentFPt.D, 2.0));
		laststep_targetspeed = 50.0 * sqrt(pow((input.Path.at(pathsize - 1).X - OldPath.at(pathsize - 2).X), 2.0) +
			pow((OldPath.at(pathsize - 1).Y - OldPath.at(pathsize - 2).Y), 2.0));
	}
	else laststep_targetspeed = currentSpeedMpS;


	int car_id;
	RoadMap PlannerMap(input);
	int check = PlannerMap.CreateRoadMap();

	//Dump some info.
	_logger->info("JMTBasedPlanner: In State: {} at Cartesian ( {:3.3f}, {:3.3f}, {:3.3f}  ) Frenet [ {:3.3f}, {:3.3f} ] going {:3.2f} mps = {:3.2f} MpH.",
	        GetStateName(), input.LocationCartesian.X, input.LocationCartesian.Y, input.LocationCartesian.ThetaRads, 
		    input.LocationFrenet.S, input.LocationFrenet.D, currentSpeedMpS, input.SpeedMpH);
	if (pathsize > 0)
	{
		_logger->info("JMTBasedPlanner: Pathsize is {}, co-ords ( {:3.3f}, {:3.3f}, {:3.3f}  ) Frenet [ {:3.3f}, {:3.3f} ] at end of Path and going {:3.2f} mps.",
			pathsize, input.Path.back().X, input.Path.back().Y, input.Path.back().ThetaRads,
			input.PathEndpointFrenet.S, input.PathEndpointFrenet.D, laststep_targetspeed);
	}
	else
	{
		_logger->info("JMTBasedPlanner: Pathsize is {}.", pathsize);
	}


	switch(EgoState)
	{
	case Uninitialized:
		//switch states to DriveInLane
		targetSpeed = Trajectory::MaxSpeedMpS;
		OutputPath = InitiateTrajectory(input);
		EgoState = DriveInLane;
		_logger->info("JMTBasedPlanner: Changing State to {} =============================== ", GetStateName());
		spdlog::get("console")->warn("JMTBasedPlanner: Changing State to{} ============================== = ", GetStateName());
		break;
	case DriveInLane:
		targetLane = currentLane; // maybe should use pathendpoint ?
		if (!TimerSet) {
			TimerSet = true;
			KeepLaneTimer = std::chrono::system_clock::now();
		}
		else
		{
			//std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - KeepLaneTimer).count() << " mS in Lane" << std::endl;

			if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - KeepLaneTimer).count() > KEEP_LANE_MINIMUM_TIME)
			{
				std::cout << "Checking for lane Change " << std::endl;
				int test = PlannerMap.CheckForLaneChange();
#ifdef DEBUG
				test = -1;//running in lane to verify straight
#endif // DEBUG
				if ( test > -1)
				{
					targetLane = test;// PlannerMap.PlanTargetLane;
					targetSpeed = Trajectory::MaxSpeedInLaneChangeMpS;
					_logger->info("JMTBasedPlanner: Initiate Lane change to lane {} with desired velocity of {:3.2f}", 
						PlannerMap.PlanTargetLane, targetSpeed);
					OutputPath = GenerateJMTLaneChangeTrajectory(input, PlannerMap.PlanTargetLane, targetSpeed, true);
					EgoState = ChangeLane;
					_logger->info("JMTBasedPlanner: Changing State to {} =============================== ", GetStateName());
					spdlog::get("console")->warn("JMTBasedPlanner: Changing State to{} ============================== = ", GetStateName());
					TimerSet = false;
					break;
				}
				_logger->info("JMTBasedPlanner: No available lane change opportunity found.");
			}
		}

		car_id = PlannerMap.CheckForSlowCarsAhead(JMTCarinfrontbuffer);
		if (car_id > -1 and (( GetAcceleration() > 0) or (GetAcceleration() < 0 and pathsize<30))) {
			//do we need to slow down
			targetSpeed = 0.995 * input.OtherCars.at(car_id).Speed2DMagnitudeMpS();
			_logger->info("JMTBasedPlanner: found Othercar id = {} at offset {} with velocity {:3.2f}",
				    input.OtherCars.at(car_id).id, car_id, input.OtherCars.at(car_id).Speed2DMagnitudeMpS());

			_logger->info("JMTBasedPlanner: sent to GenerateKeepLane with DesiredSpeed of {:3.2f}", targetSpeed);
			OutputPath = GenerateKeepInLaneTrajectory(input, targetSpeed, true);
			break;
		}

		if (pathsize < 40) //(about every 2nd to 5th iteration)
		{
			targetSpeed = Trajectory::MaxSpeedMpS;
			_logger->info("JMTBasedPlanner: send to GenerateKeepLane because only {} tracking points, DesiredVelocity = {}", pathsize, targetSpeed);
			//reset to truncate to reduce offset errors.
			OutputPath = GenerateKeepInLaneTrajectory(input, targetSpeed, true);
		}else{
			_logger->info("JMTBasedPlanner: just use existing KeepInLane Path");
			OutputPath = { OldPath };
		}
		break;

	case ChangeLane:
		if (currentLane == targetLane and input.LocationFrenet.IsAtCenterofLane(targetLane)) {
			//pathplan forward at velocity, and change state to 
			EgoState = DriveInLane;
			_logger->info("JMTBasedPlanner: Changing State to {} ===============================", GetStateName());
			spdlog::get("console")->warn("JMTBasedPlanner: Changing State to{} ============================== = ", GetStateName());
		}
		if (input.Path.size() < 40) {
			//reinitiate move to target lane from current location - this probably needs to deal with D velocity....

			if (input.LocationFrenet.GetLane() == targetLane)
			{
				targetSpeed = Trajectory::MaxSpeedMpS;
				car_id = PlannerMap.CheckForSlowCarsAhead(JMTCarinfrontbuffer);
				if (car_id > -1 and ((GetAcceleration() > 0) or (GetAcceleration() < 0 and pathsize < 30)))
				{
					//do we need to slow down
					targetSpeed = 0.995 * input.OtherCars.at(car_id).Speed2DMagnitudeMpS();
					_logger->info("JMTBasedPlanner: found Othercar id = {} at offset {} with velocity {:3.2f}",
						input.OtherCars.at(car_id).id, car_id, input.OtherCars.at(car_id).Speed2DMagnitudeMpS());
				}
				_logger->info("JMTBasedPlanner: send to GenerateKeepInLane because in TargetLane {} with Pathsize of {} and DesiredVelocity = {}", targetLane, pathsize, targetSpeed);
				OutputPath = GenerateKeepInLaneTrajectory(input, targetSpeed);
			}
			targetSpeed = Trajectory::MaxSpeedInLaneChangeMpS;
			_logger->info("JMTBasedPlanner: send to GenerateJMTLaneChangeTraj because not in TargetLane {} with Pathsize of {} and DesiredVelocity = {}", targetLane, pathsize, targetSpeed);
			OutputPath = GenerateJMTLaneChangeTrajectory(input, targetLane, targetSpeed, true);
		}
		else 
		{
			_logger->info("JMTBasedPlanner:just use existing ChangeLane Path");
			OutputPath = { OldPath };
		}

	}
	return OutputPath;
}

