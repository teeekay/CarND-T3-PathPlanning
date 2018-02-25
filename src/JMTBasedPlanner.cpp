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


//#define DEBUG


// if we plan 50 points ahead at maximum speed of 49.5 mph = 22.1 mps, and at 0.02 sec intervals, we will only go out 22.1 m.


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
	}
	return (char *)"Invalid - check GetStateName function";
}


std::vector<CartesianPoint> JMTBasedPlanner::GeneratePath(PathPlannerInput input)
{
	
	//get a sorted list of cars in this lane in front of car that will be within 30 m.
	std::vector <CartesianPoint> OutputPath;
	std::vector <CartesianPoint> OldPath = input.Path;
	if (input.SpeedMpS > MaxVelocityReported)
	{
		MaxVelocityReported = input.SpeedMpS;
		if (MaxVelocityReported > Trajectory::MaxSpeedMpS * 0.995)
		{
			_JPL->warn("Maximum Velocity of {:3.3f} mps or {:3.3f} mph recorded at ({:3.3f},{:3.3f}).", MaxVelocityReported, input.SpeedMpH, input.LocationCartesian.X, input.LocationCartesian.Y);
			spdlog::get("console")->warn("Maximum Velocity of {:3.3f} mps or {:3.3f} mph recorded at ({:3.3f},{:3.3f}).", MaxVelocityReported, input.SpeedMpH, input.LocationCartesian.X, input.LocationCartesian.Y);
		}
	}
	int pathsize = OldPath.size();// > 5) ? 5 : input.Path.size();
	currentLane = input.LocationFrenet.GetLane();
	currentSpeedMpS = input.SpeedMpS;

	if (pathsize > 2) {
		laststep_targetspeed = 50.0 * sqrt(pow((input.Path.at(pathsize - 1).X - OldPath.at(pathsize - 2).X), 2.0) +
			pow((OldPath.at(pathsize - 1).Y - OldPath.at(pathsize - 2).Y), 2.0));
	}
	else laststep_targetspeed = currentSpeedMpS;


	int car_id, car2_id;
	RoadMap PlannerMap(map, input);
	int check = PlannerMap.CreateRoadMap();

	//_JPL->info("In State: {}  in Lane {} at Cartesian ( {:3.3f}, {:3.3f}, {:3.3f}  ) Frenet [ {:3.3f}, {:3.3f} ] going {:3.2f} mps = {:3.2f} MpH.",
	_JPL->info("In State: {}  in Lane {} at Cartesian ( {:3.3f}, {:3.3f}, {:3.3f}  ) {} going {:3.2f} mps = {:3.2f} MpH.",
	        GetStateName(), input.LocationFrenet.GetLane(), input.LocationCartesian.X, input.LocationCartesian.Y, input.LocationCartesian.ThetaRads,
		    input.LocationFrenet, //input.LocationFrenet.S, input.LocationFrenet.D,
		currentSpeedMpS, input.SpeedMpH);
	_JPL->flush( );
	if (pathsize > 0)
	{
	
		_JPL->info("Pathsize is {}, At end of Path in Lane {} co-ords ( {:3.3f}, {:3.3f}, {:3.3f}  ) Frenet [ {:3.3f}, {:3.3f} ] with speed {:3.2f} mps.",
			pathsize, input.PathEndpointFrenet.GetLane(), input.Path.back().X, input.Path.back().Y, input.Path.back().ThetaRads,
			input.PathEndpointFrenet.S, input.PathEndpointFrenet.D, laststep_targetspeed);
		//Note: PathEndpointFrenet is not reliable when a way marker is between the car and the endpoint
	}
	else
	{
		_JPL->info("Pathsize is {}.", pathsize);
	}


	switch(EgoState)
	{
	case Uninitialized:
		//switch states to DriveInLane once moving
		if (pathsize == 0)
		{
			_JPL->info("In State {} =============================== ", GetStateName( ));
			spdlog::get("console")->warn("In State {} ============================== = ", GetStateName( ));
			targetSpeed = Trajectory::MaxSpeedMpS;
			OutputPath = InitiateTrajectory(input);
		}
		else
		{
			OutputPath = { OldPath };
			if (pathsize < 20)
			{
				EgoState = DriveInLane;
				_JPL->info("Changing State to {} =============================== ", GetStateName( ));
				spdlog::get("console")->warn("Changing State to {} ============================== = ", GetStateName( ));
			}
		}
		break;
	case DriveInLane:
		targetLane = currentLane;
		if (!TimerSet) {
			// just to be a good driver, stay in lane for specified time before moving lanes.
			TimerSet = true;
			KeepLaneTimer = std::chrono::system_clock::now();
		}
		else
		{
			if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - KeepLaneTimer).count() > KEEP_LANE_MINIMUM_TIME)
			{
				_JPL->info("Checking for lane Change ");
				int test = PlannerMap.CheckForLaneChange();
				if ( test > -1)
				{
					targetLane = PlannerMap.PlanTargetLane;
					car2_id = PlannerMap.CheckForSlowCarInOtherLane(targetLane, 2*JMTCarinfrontbuffer);
					if (car2_id > -1)
					{
					
						double gap = RangeS(map.CartesianToFrenet(input.OtherCars.at(car2_id).LocationCartesian).S - input.LocationFrenet.S);
						targetSpeed = 0.98 * input.OtherCars.at(car2_id).Speed2DMagnitudeMpS( );  
						if (targetSpeed > Trajectory::MaxSpeedMpS) targetSpeed = Trajectory::MaxSpeedMpS;
						_JPL->info("found Othercar id = {} in targetLane {} with velocity {:3.2f} at distance of {:3.2f} m.",
							input.OtherCars.at(car2_id).id, targetLane, input.OtherCars.at(car2_id).Speed2DMagnitudeMpS( ), gap);
						if (targetSpeed > Trajectory::MaxSpeedInLaneChangeMpS) targetSpeed = Trajectory::MaxSpeedInLaneChangeMpS;
					
					} else 
						targetSpeed = Trajectory::MaxSpeedInLaneChangeMpS;
					
					_JPL->info("Initiate Lane change from Lane {} to Lane {} with desired velocity of {:3.2f} ", 
						currentLane, PlannerMap.PlanTargetLane, targetSpeed);
					OutputPath = GenerateJMTLaneChangeTrajectory(input, PlannerMap.PlanTargetLane, targetSpeed, true);
					EgoState = ChangeLane;
					_JPL->info("Changing State to {} =============================== ", GetStateName());
					spdlog::get("console")->warn("Changing State to {} ============================== = ", GetStateName());
					TimerSet = false;
					break;
				}
				_JPL->info("No available lane change opportunity found.");
			}
		}

		car_id = PlannerMap.CheckForSlowCarsAhead(JMTCarinfrontbuffer);
		if (car_id > -1){ 
			//switched to using car cartesian, because  sometimes Frenet co-ords received are corrupted (near S=0)
			double gap = RangeS(map.CartesianToFrenet(input.OtherCars.at(car_id).LocationCartesian).S - input.LocationFrenet.S); // from HighwayMap
			//do we need to slow down
			if (DeccelerationLoopCounter > 0 and pathsize > 20 )// remove req for centerlane and input.PathEndpointFrenet.IsAtCenterofLane(targetLane))
			{
				_JPL->info("found Othercar id = {} with velocity {:3.2f} at distance {:3.2f} m but staying on Decceleration path for now.",
					input.OtherCars.at(car_id).id, input.OtherCars.at(car_id).Speed2DMagnitudeMpS(), gap);
				DeccelerationLoopCounter--;
				OutputPath = { OldPath };
				break;
			}
			targetSpeed = 0.95 * input.OtherCars.at(car_id).Speed2DMagnitudeMpS();  //was .99
			if (targetSpeed > Trajectory::MaxSpeedMpS) targetSpeed = Trajectory::MaxSpeedMpS;
			_JPL->info("found Othercar id = {} with velocity {:3.2f} at distance of {:3.2f} m.",
				    input.OtherCars.at(car_id).id, input.OtherCars.at(car_id).Speed2DMagnitudeMpS(), gap);

			_JPL->info("sent to GenerateKeepLane to prevent rear-ending with DesiredSpeed of {:3.2f}", targetSpeed);
			OutputPath = GenerateKeepInLaneTrajectory(input, targetSpeed, true);
			DeccelerationLoopCounter = 20; //20
			break;
		}
		else DeccelerationLoopCounter = 0;

		if (pathsize < 40) //(about every 2nd to 5th iteration)
		{
			targetSpeed = Trajectory::MaxSpeedMpS;
			_JPL->info("send to GenerateKeepLane because only {} tracking points, DesiredVelocity = {:3.2f} ", pathsize, targetSpeed);
			//reset to truncate to reduce offset errors.
			OutputPath = GenerateKeepInLaneTrajectory(input, targetSpeed, true);
		}
        else{
			_JPL->info("just use existing KeepInLane Path");
			OutputPath = { OldPath };
		}
		break;

	case ChangeLane:
		if (currentLane == targetLane and input.LocationFrenet.IsAtCenterofLane(targetLane)) {
			//pathplan forward at velocity, and change state to 
			EgoState = DriveInLane;
			_JPL->info("Changing State to {} ===============================", GetStateName());
			spdlog::get("console")->warn("Changing State to {} ============================== = ", GetStateName());
		}
		if (input.Path.size() < 20) {
			//reinitiate move to target lane from current location - this probably needs to deal with D velocity....

			if (input.LocationFrenet.GetLane() == targetLane)
			{
				targetSpeed = Trajectory::MaxSpeedMpS;
				car_id = PlannerMap.CheckForSlowCarsAhead(JMTCarinfrontbuffer);
				if (car_id > -1 and ((GetAcceleration() > 0) or (GetAcceleration() < 0 and pathsize < 30)))
				{
					//do we need to slow down
					targetSpeed = 0.90 * input.OtherCars.at(car_id).Speed2DMagnitudeMpS();  //safety factor increased here
					_JPL->info("found Othercar id = {} at offset {} with velocity {:3.2f}",
						input.OtherCars.at(car_id).id, car_id, input.OtherCars.at(car_id).Speed2DMagnitudeMpS());
				}
				_JPL->info("send to GenerateKeepInLane because in TargetLane {} with Pathsize of {} and DesiredVelocity = {:3.2f}", targetLane, pathsize, targetSpeed);
				OutputPath = GenerateKeepInLaneTrajectory(input, targetSpeed, false);
			}
			else
			{
				targetSpeed = Trajectory::MaxSpeedInLaneChangeMpS;
				_JPL->info("send to GenerateJMTLaneChangeTraj because not in TargetLane {} with Pathsize of {} and DesiredVelocity = {:3.2f}", targetLane, pathsize, targetSpeed);
				OutputPath = GenerateJMTLaneChangeTrajectory(input, targetLane, targetSpeed, true);
			}
		}
		else 
		{
			_JPL->info("JMTBasedPlanner:just use existing ChangeLane Path");
			OutputPath = { OldPath };
		}

	}
	return OutputPath;
}