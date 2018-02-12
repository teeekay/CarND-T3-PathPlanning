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
#define G_FORCE_MPS 9.8
// maximum safe acceleration per step

//const double DefaultAcceleration = 0.2;// .224;
const double MaxFwdAccelerationMpSsq = 0.25 * G_FORCE_MPS; /* 0.25 times force of gravity */ //0.25 still exceeds max deceleration when approaching slow cars
const double MaxBrakingAccelerationMpSsq = -0.25 * G_FORCE_MPS; /* 0.25 times force of gravity */ //0.2 still exceeds max deceleration when approaching slow cars


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


std::vector<CartesianPoint> JMTBasedPlanner::GeneratePath(PathPlannerInput input)
{
	
	//get a sorted list of cars in this lane in front of car that will be within 30 m.
	std::vector <CartesianPoint> OutputPath;
	std::vector <CartesianPoint> OldPath = input.Path;
	if (input.SpeedMpS > MaxVelocityReported)
	{
		MaxVelocityReported = input.SpeedMpS;
		if (MaxVelocityReported > MaxSpeedMpS)
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

	

	std::cout << "In State: " << GetStateName() << " at Cartesian Co-ords (" << input.LocationCartesian.X << ", " << input.LocationCartesian.Y << ", " << input.LocationCartesian.ThetaRads 
		<< ") going " << currentSpeedMpS << " mps, with speed in Frenet of " << currentFSpeedMPS << " mps and "<< input.SpeedMpH << " MpH." << std::endl;
	std::cout << " Pathsize is " << pathsize  << " Speed at end Path is " << laststep_targetspeed << " mps." << std::endl;

	
//	if(pathsize>0){
//		std::cout << "Debug PATH - [i,S,D] - ";
//		
//		for (int i = 0; i < 5 and i < pathsize; i++)	std::cout << " [" << i << " , " << map.CartesianToFrenet(input.Path.at(i)).S 
//			<< " , " << map.CartesianToFrenet(input.Path.at(i)).D << "]";
//		std::cout << std::endl;
//	}



	switch(EgoState)
	{
	case Uninitialized:
		//switch states to DriveInLane
		targetSpeed = MaxSpeedMpS;
		OutputPath = InitiateTrajectory(PlannerMap, input);
		EgoState = DriveInLane;
		std::cout << "Changing State to " << GetStateName() << "===============================" << std::endl;
		break;
	case DriveInLane:
		targetLane = currentLane; // maybe should use pathendpoint ?
		if (!TimerSet) {
			TimerSet = true;
			KeepLaneTimer = std::chrono::system_clock::now();
		}
		else
		{
			std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - KeepLaneTimer).count() << " mS in Lane" << std::endl;

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
					std::cout << "Lane change to " << test << "or to " << PlannerMap.PlanTargetLane << std::endl;
					OutputPath = GenerateJMTLaneChangeTrajectory(PlannerMap, input);
					EgoState = ChangeLane;
					std::cout << "Changing State to " << GetStateName() << "===============================" << std::endl;
					TimerSet = false;
					break;
				}
				std::cout << "No available lane change opportunity " << std::endl;
			}
		}

		car_id = PlannerMap.CheckForSlowCarsAhead(JMTCarinfrontbuffer);
		if (car_id > -1 and (( acceleration > 0) or (acceleration < 0 and pathsize<30))) {
			//do we need to slow down
			targetSpeed = 0.95 * input.OtherCars.at(car_id).Speed2DMagnitudeMpS();
			std::cout << "found Othercar id = " << input.OtherCars.at(car_id).id << " at offset " << car_id 
				<< " with velocity " << input.OtherCars.at(car_id).Speed2DMagnitudeMpS() << std::endl;

			std::cout << "sent to GenerateKeepLane because of slow car ahead" << std::endl;
			
			OutputPath = GenerateKeepInLaneTrajectory(PlannerMap, input);
			break;
		}

		if (pathsize < 40) //(about every 2nd to 5th iteration)
		{
			std::cout << "send to GenerateKeepLane because < 40 tracking points" << std::endl;
			targetSpeed = MaxSpeedMpS;

			OutputPath = GenerateKeepInLaneTrajectory(PlannerMap, input);
		}else{
			std::cout << "just use existing Path" << std::endl;
			OutputPath = { OldPath };
		}
		break;

	case ChangeLane:
		if (currentLane == targetLane and input.LocationFrenet.IsAtCenterofLane(targetLane)) {
			//pathplan forward at velocity, and change state to 
			EgoState = DriveInLane;
			std::cout << "Changing State to " << GetStateName() << "===============================" << std::endl;
		}
		if (input.Path.size() < 40) {
			//reinitiate move to target lane from current location
			if (input.LocationFrenet.GetLane() == targetLane)
			{
				OutputPath = GenerateKeepInLaneTrajectory(PlannerMap, input);
			}
			OutputPath = GenerateJMTLaneChangeTrajectory(PlannerMap, input);
		}
		else 
		{
			OutputPath = { OldPath };
		}

	}

	
	return OutputPath;
}


double JMTBasedPlanner::safeAcceleration(double accel) {
	return ((accel >= 0) ? (accel> MaxFwdAccelerationMpSsq) ? MaxFwdAccelerationMpSsq : accel :
		(accel < MaxBrakingAccelerationMpSsq) ? MaxBrakingAccelerationMpSsq : accel);
}

std::vector<CartesianPoint> JMTBasedPlanner::InitiateTrajectory(RoadMap PlannerMap, PathPlannerInput input)
{
	FrenetPoint ReferenceFPt = input.LocationFrenet;
	CartesianPoint ReferenceCPt = input.LocationCartesian;
	_logger->info("in InitiateTrajectory");

	double T = 1.0;
	
	double displacement = 0.5 * MaxFwdAccelerationMpSsq * pow(T, 2.0);
	double targetSpeed = MaxFwdAccelerationMpSsq * T;

	FrenetPoint EndFPt = { ReferenceFPt.S + displacement, ReferenceFPt.D };

	std::vector<CartesianPoint> StartupPath;
	
	std::vector<double> S_NowState = { ReferenceFPt.S, 0.0, 0.0 };
	std::vector<double> D_NowState = { ReferenceFPt.D, 0.0, 0.0 };

	std::vector<double> S_EndState = { EndFPt.S, targetSpeed, 0.0 };
	std::vector<double> D_EndState = { EndFPt.D, 0.0, 0.0 };

	JMT Jerk = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
	FrenetPoint FPt;
	CartesianPoint CPt;
	//_logger->info("t,S,D,Vel_S,Vel_D,Acc_S,Acc_D,Jerk_S,Jerk_D");
	for (double t = 0.0; t <= T; t += SimulatorRunloopPeriod) {
		FPt = Jerk.JMTDisplacementAt(t);
		CPt = map.FrenetToCartesian(FPt);
		if (t < SimulatorRunloopPeriod) {
			CPt.ThetaRads = ReferenceCPt.ThetaRads;
		}
		else {
			CPt.ThetaRads = atan2(CPt.Y - StartupPath.back().Y, CPt.X - StartupPath.back().X);
		}
		StartupPath.push_back(CPt);
	}

	//offset_path by any discrepencies from FrenetToCartesian from known location
	CartesianPoint offset = { StartupPath.at(0).X - input.LocationCartesian.X,
		StartupPath.at(0).Y - input.LocationCartesian.Y };

	for (int i = 0; i<StartupPath.size(); i++)
	{
		StartupPath.at(i).X -= offset.X;
		StartupPath.at(i).Y -= offset.Y;
	}

	_logger->info("Checking Initialization Path.");
	Acc_Jerk PathTest = CheckPath(StartupPath, SimulatorRunloopPeriod, input.SpeedMpS);
	_logger->info("Check Path finds Maximum Velocity in Path is {} ", PathTest.Max_Vel);
	_logger->info("Check Path finds Acceleration Maximum = {0} Fwd = {1} Lateral = {2}  ",
		PathTest.Max_Total_Acc, PathTest.Max_Fwd_Acc, PathTest.Max_Lat_Acc);
	_logger->info("Check Path finds Jerk Maximum = {0} Fwd = {1} Lateral = {2}  ",
		PathTest.Max_Total_Jerk , PathTest.Max_Fwd_Jerk, PathTest.Max_Lat_Jerk);
	_logger->flush();
	
	LastIterEndpointFPt = map.CartesianToFrenet(StartupPath.back());

	return StartupPath;
}


std::vector<CartesianPoint> JMTBasedPlanner::GenerateKeepInLaneTrajectory(RoadMap PlannerMap, PathPlannerInput input)
{
	std::cout << "In GenerateKeepInLaneTrajectory" << std::endl;

	assert(-1 < currentLane < 3); // Make sure in range

	int LanesChange = targetLane - currentLane;
	assert(abs(LanesChange) < 1);  // 1 lane at a time

								   //	std::cout << "Path Size is " << input.Path.size() << std::endl;

	CartesianPoint ReferenceCPt;
	FrenetPoint ReferenceFPt;
	CartesianPoint PrevCPt;


	int pathsize = 10;  //want to keep a buffer of existing pathpoints to account for any delays sending path (so there are still old pathpoints in front of car)
	if (input.PreviousPathX.size() > pathsize)
	{
		ReferenceCPt = input.Path.at(pathsize);
		input.PreviousPathX.resize(pathsize);
		input.PreviousPathY.resize(pathsize);
		input.Path.resize(pathsize);
		PrevCPt = input.Path.at(pathsize - 1);
		ReferenceCPt.ThetaRads = atan2(ReferenceCPt.Y - PrevCPt.Y, ReferenceCPt.X - PrevCPt.X);
		ReferenceFPt = map.CartesianToFrenet(ReferenceCPt);
		//input.PathEndpointFrenet = map.CartesianToFrenet(ReferenceCPt);
	}
	if (input.PreviousPathX.size() == 0)
	{
		//input.PathEndpointFrenet = input.LocationFrenet;
		ReferenceFPt = map.CartesianToFrenet(ReferenceCPt);
		double delta_t = 0.02;
		ReferenceCPt = input.LocationCartesian;
		PrevCPt = { input.LocationCartesian.X - delta_t * cos(input.LocationCartesian.ThetaRads),
			input.LocationCartesian.Y - delta_t * sin(input.LocationCartesian.ThetaRads) };
	}

	//T is time for maneuver - when staying in lane, keep it to 1 second intervals
	double T = 1.5;

	
	double tmp_targetSpeed = MaxSpeedMpS;
	if (targetSpeed < laststep_targetspeed)
	{
		tmp_targetSpeed = targetSpeed;
	}
	//

	if (input.Path.size() > 1)
	{
//		FrenetPoint CurrentFPt = map.CartesianToFrenet(input.Path.at(input.Path.size() - 2));
//		FrenetPoint PathStartFPt = map.CartesianToFrenet(input.Path.at(input.Path.size() - 1));
//		laststep_targetspeed = 50.0 * sqrt(pow(PathStartFPt.S - CurrentFPt.S, 2.0) + pow(PathStartFPt.D - CurrentFPt.D, 2.0));
		laststep_targetspeed = 50.0 * sqrt(pow((input.Path.at(input.Path.size() - 1).X
			- input.Path.at(input.Path.size() - 2).X), 2.0) +
			pow((input.Path.at(input.Path.size() - 1).Y
				- input.Path.at(input.Path.size() - 2).Y), 2.0));
	}
	else
	{
		//laststep_targetspeed = currentFSpeedMPS;
		laststep_targetspeed = currentSpeedMpS;
	}


	acceleration = safeAcceleration((tmp_targetSpeed - laststep_targetspeed) / T);
	targetSpeed = laststep_targetspeed + (T * acceleration);


	double displacement = laststep_targetspeed * T + 0.5 * acceleration * pow(T, 2.0);
	std::cout << "targetspeed of " << targetSpeed << " calculated with Safe acceleration of " << acceleration << " giving displacement of " << displacement << "m." << std::endl;

	FrenetPoint EndFPt;
	EndFPt.S = ReferenceFPt.S + displacement;
	EndFPt.D = ReferenceFPt.LaneCenterDCoord(ReferenceFPt.GetLane());

	std::cout << "Staying in Lane " << ReferenceFPt.GetLane() << ", with acceleration of " << acceleration
		<< " at S = " << input.PathEndpointFrenet.S << std::endl;
	
	double startspeed = laststep_targetspeed;

	if (acceleration < 0)
	{
		startspeed = laststep_targetspeed + acceleration * SimulatorRunloopPeriod;
	}

	std::vector<double> S_NowState = { ReferenceFPt.S, startspeed, 0.0 };
	std::vector<double> D_NowState = { ReferenceFPt.D, 0.0, 0.0 };

	std::vector<double> S_EndState = { EndFPt.S , targetSpeed, 0.0 };
	std::vector<double> D_EndState = { EndFPt.LaneCenterDCoord(targetLane), 0.0, 0.0 };

	JMT Jerk = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
	FrenetPoint FPt;
	CartesianPoint CPt;
	std::vector<CartesianPoint> StayInLanePath;

//	_logger->info("t,S,D,Vel_S,Vel_D,Acc_S,Acc_D,Jerk_S,Jerk_D");

	bool ValidPath = false;
	double vel_adj = 0.0;
	double Acc_adj = 0.0;
	int counter = 0;
	while (!ValidPath)

	{
		std::vector<FrenetPoint> FPath;
		for (double t = 0.0; t <= T; t += SimulatorRunloopPeriod)
		{
			FPt = Jerk.JMTDisplacementAt(t);
			FPath.push_back(FPt);
		}
		StayInLanePath = map.ConvertCurveMaintainSpeed(FPath, ReferenceCPt);
		
		_logger->info("Checking StayinLanePath. counter = {}",counter);
		Acc_Jerk PathTest = CheckPath(StayInLanePath, SimulatorRunloopPeriod, input.SpeedMpS);
		_logger->info("Check Path finds Maximum Velocity in Path is {} while MaxSpeedMPS is {} ", PathTest.Max_Vel, MaxSpeedMpS);
		_logger->info("Check Path finds Acceleration Maximum = {0} Fwd = {1} Lateral = {2}  ",
			PathTest.Max_Total_Acc, PathTest.Max_Fwd_Acc, PathTest.Max_Lat_Acc);
		_logger->flush();

		if ((PathTest.Max_Vel > MaxSpeedMpS) and (acceleration > -1.0))
		{
			
			//T += 0.1;
			std::cout << "Recalculating Path because Max_Vel of " << PathTest.Max_Vel << "exceeds Max velocity of " << MaxSpeedMpS << " mps. Dumping Path for debug " << std::endl;
			for (auto & lCPt : StayInLanePath) std::cout << lCPt.X << ", " << lCPt.Y << ", " << lCPt.ThetaRads << std::endl;

			double vel_diff = PathTest.Max_Vel - MaxSpeedMpS > 1.0? (PathTest.Max_Vel - MaxSpeedMpS)*1.5: 1.0;
			targetSpeed -= vel_diff;// (PathTest.Max_Vel - targetSpeed + 0.1);
			acceleration = safeAcceleration((targetSpeed - laststep_targetspeed) / T);
			if (acceleration < 0)
			{
				startspeed = laststep_targetspeed + acceleration * SimulatorRunloopPeriod;
				S_NowState = { ReferenceFPt.S, startspeed, 0.0 };
			}
			targetSpeed = laststep_targetspeed + (T * acceleration);
			displacement = laststep_targetspeed * T + 0.5 * acceleration * pow(T, 2.0);
			EndFPt.S = ReferenceFPt.S + displacement;
			S_EndState = { EndFPt.S , targetSpeed, 0.0 };
			Jerk = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
			std::cout << "targetspeed of " << targetSpeed << " calculated with Safe acceleration of " << acceleration << " giving displacement of " << displacement << "m." << std::endl;
			counter++;
			if (counter > 5)
			{
				ValidPath = true;
			}
			else
			{
				StayInLanePath.clear();
			}
		}
		else
		{
			ValidPath = true;
		}

	}

	


	//offset_path by any discrepencies from FrenetToCartesian from known location
	CartesianPoint offset = { StayInLanePath.at(0).X - ReferenceCPt.X,
		StayInLanePath.at(0).Y - ReferenceCPt.Y };

	for (auto & lCPt: StayInLanePath)
//		int i = 0; i<StayInLanePath.size(); i++)
	{
		//StayInLanePath.at(i).X -= offset.X;
		//StayInLanePath.at(i).Y -= offset.Y;
		lCPt.X -= offset.X;
		lCPt.Y -= offset.Y;
	}


	if( input.Path.size() > 0)
	  StayInLanePath.insert(StayInLanePath.begin(), input.Path.begin(), input.Path.end());


	for (auto lCPt : StayInLanePath) std::cout << lCPt.X << ", " << lCPt.Y << ", " << lCPt.ThetaRads << std::endl;
	LastIterEndpointFPt = map.CartesianToFrenet(StayInLanePath.back());
	return StayInLanePath;
}

std::vector<CartesianPoint> JMTBasedPlanner::GenerateJMTLaneChangeTrajectory(RoadMap PlannerMap, PathPlannerInput input)
{

	assert(-1 < targetLane < 3); // Make sure in range

	int LanesChange = targetLane - currentLane;
	assert(abs(LanesChange) < 2);  // 1 lane at a time

	CartesianPoint ReferenceCPt;
	FrenetPoint ReferenceFPt;
	CartesianPoint PrevPoint;
	CartesianPoint LaneChangeStartCPt;

	if (input.Path.size() == 0)
	{
		input.PathEndpointFrenet = input.LocationFrenet;
		double delta_t = 1.0; // stick the previous point back a while
		ReferenceCPt = input.LocationCartesian;
		ReferenceFPt = map.CartesianToFrenet(ReferenceCPt);
		double SpeedMpS = fabs(input.SpeedMpS) < 0.1 ? 1.0 : input.SpeedMpS;  //deal with the fact the car may be stopped at present
		PrevPoint = { ReferenceCPt.X - delta_t * SpeedMpS * cos(ReferenceCPt.ThetaRads),
			ReferenceCPt.Y - delta_t * SpeedMpS * sin(ReferenceCPt.ThetaRads) };
		delta_t = SimulatorRunloopPeriod;
		LaneChangeStartCPt = { ReferenceCPt.X + delta_t * SpeedMpS * cos(ReferenceCPt.ThetaRads),
			ReferenceCPt.Y + delta_t * SpeedMpS * sin(ReferenceCPt.ThetaRads) };

	}
	else
	{
		int DesiredSize = 10;
		if (input.Path.size() <= DesiredSize) // try to put a straight path for 0.2 seconds, then we'll add the lane change.  //watch out on a curve!
		{
			double delta_t = SimulatorRunloopPeriod;
			double yaw = input.LocationCartesian.ThetaRads;
			double SpeedMpS = fabs(input.SpeedMpS) < 0.1 ? 1.0 : input.SpeedMpS;
			CartesianPoint CPt;
			for (int i = input.Path.size(); i < DesiredSize + 1; i++)
			{
				CPt = { input.Path.back().X + delta_t * SpeedMpS * cos(yaw),
					input.Path.back().Y + delta_t * SpeedMpS * sin(yaw), yaw };
				input.Path.push_back(CPt);
			}
			std::cout << "Extended input.Path to Size " << input.Path.size() << std::endl;
		}

		//if (input.Path.size() > 10) {// we want to start the turn as soon as possible so the lanechange assumptions remain valid
		// but we want to make sure that we allow for delays

		LaneChangeStartCPt = { input.Path.at(DesiredSize).X,input.Path.at(DesiredSize).Y };
		input.PreviousPathX.resize(DesiredSize);
		input.PreviousPathY.resize(DesiredSize);
		input.Path.resize(DesiredSize);
		ReferenceCPt = input.Path.at(DesiredSize - 1);
		ReferenceFPt = map.CartesianToFrenet(ReferenceCPt);
		PrevPoint = input.Path.at(DesiredSize - 2);
		ReferenceCPt.ThetaRads = atan2(ReferenceCPt.Y - PrevPoint.Y, ReferenceCPt.X - PrevPoint.X);


	}

	//T is time for Lanechange
	//start here, but increase T if jerk/acceleration exceeds allowable
	double T = 1.7;

	//FrenetPoint CurrentFPt = map.CartesianToFrenet(input.Path.at(input.Path.size() - 2));
	//FrenetPoint PathStartFPt = map.CartesianToFrenet(input.Path.at(input.Path.size() - 1));
	//laststep_targetspeed = 50.0 * sqrt(pow(PathStartFPt.S - CurrentFPt.S, 2.0) + pow(PathStartFPt.D - CurrentFPt.D, 2.0));

	laststep_targetspeed = 50.0 * sqrt(pow(LaneChangeStartCPt.X - ReferenceCPt.X, 2.0) +
		pow(LaneChangeStartCPt.Y - ReferenceCPt.Y, 2.0));

	//set the speed to come out of the maneuver 
	double tmp_targetSpeed;//
	if (laststep_targetspeed >= MaxSpeedInLaneChangeMpS)
	{
		tmp_targetSpeed = MaxSpeedInLaneChangeMpS; //reduce speed slightly in lane change)
	}
	else tmp_targetSpeed = MaxSpeedInLaneChangeMpS;



	acceleration = safeAcceleration((tmp_targetSpeed - laststep_targetspeed) / T);

	laststep_targetspeed = laststep_targetspeed + acceleration * SimulatorRunloopPeriod;

	//acceleration = 0.0; //just to test
	targetSpeed = laststep_targetspeed + (T * acceleration);

	double displacement = laststep_targetspeed * T + 0.5 * acceleration * pow(T, 2.0);
	std::cout << "targetspeed of " << targetSpeed << " calculated with Safe acceleration of " << acceleration << " giving displacement of " << displacement << "m." << std::endl;


	std::cout << "Initiating Lane Change from Lane " << currentLane << " to Lane " << targetLane << ", with acceleration of " << acceleration << std::endl;
	std::cout << "FrenetPoints in Path are:" << std::endl;
	for (auto&p : input.Path)
	{

		std::cout << "( " << p.X << ", " << p.Y << ", " << p.ThetaRads << " ) [ " << map.CartesianToFrenet(p).S << ", " << map.CartesianToFrenet(p).D << " ]" << std::endl;
	}

	FrenetPoint LaneChangeStartFPt = (map.CartesianToFrenet(LaneChangeStartCPt));

	std::cout << "Located Lane Change start Co,ords at (" << LaneChangeStartCPt.X << ", " << LaneChangeStartCPt.Y
		<< ") or [" << LaneChangeStartFPt.S << ", " << LaneChangeStartFPt.D << "]" << std::endl;

	//std::vector<double> S_NowState = { 0.0, laststep_speed, 0.0 };
	//std::vector<double> D_NowState = { 0.0, 0.0, 0.0 };
///////
	double startspeed = laststep_targetspeed;

	if (acceleration < 0)
	{
		startspeed = laststep_targetspeed + acceleration * SimulatorRunloopPeriod;
	}
////////
	double LaneChangeFactor = ((targetSpeed + startspeed) / 2.0 * T) / sqrt(pow(T*(targetSpeed + laststep_targetspeed) / 2.0, 2.0) + pow(3.9, 2.0));
	double DistanceForManeuver = (targetSpeed + startspeed) / 2.0 * T * LaneChangeFactor;

	FrenetPoint LaneChangeEndFPt = { LaneChangeStartFPt.S + DistanceForManeuver,	FrenetPoint::LaneCenterDCoord(targetLane) };
	CartesianPoint LaneChangeEndCPt = map.FrenetToCartesian(LaneChangeEndFPt);

	std::cout << "Located Lane Change end Co,ords at (" << LaneChangeEndCPt.X << ", " << LaneChangeEndCPt.Y
		<< ") or [" << LaneChangeEndFPt.S << ", " << LaneChangeEndFPt.D << "]" << std::endl;

	std::vector<double> S_NowState = { LaneChangeStartFPt.S, startspeed, 0.0 };
	std::vector<double> D_NowState = { LaneChangeStartFPt.D, 0.0, 0.0 };

	std::vector<double> S_EndState = { LaneChangeEndFPt.S, targetSpeed, 0.0 };
	std::vector<double> D_EndState = { LaneChangeEndFPt.D, 0.0, 0.0 };

	JMT Jerk = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
	FrenetPoint FPt;
	CartesianPoint CPt;
	std::vector<CartesianPoint> ChangeLanePath;
	std::vector<FrenetPoint> FPath;
	////////
	bool ValidPath = false;
	double vel_adj = 0.0;
	double Acc_adj = 0.0;
	int counter = 0;
	while (!ValidPath)
	{

		//	_logger->info("t,S,D,Vel_S,Vel_D,Acc_S,Acc_D,Jerk_S,Jerk_D");

		FPath.clear();
		for (double t = 0.0; t <= T; t += SimulatorRunloopPeriod)
		{
			FPt = Jerk.JMTDisplacementAt(t);
			FPath.push_back(FPt);
		}
		ChangeLanePath = map.ConvertCurveMaintainSpeed(FPath, ReferenceCPt);
		_logger->info("Checking ChangeLanePath. counter = {}", counter);
		Acc_Jerk PathTest = CheckPath(ChangeLanePath, SimulatorRunloopPeriod, input.SpeedMpS);
		_logger->info("Check Path finds Maximum Velocity in Path is {} while MaxSpeedMPS is {} ", PathTest.Max_Vel, MaxSpeedMpS);
		_logger->info("Check Path finds Acceleration Maximum = {0} Fwd = {1} Lateral = {2}  ",
			PathTest.Max_Total_Acc, PathTest.Max_Fwd_Acc, PathTest.Max_Lat_Acc);
		_logger->flush();

		if ((PathTest.Max_Vel > MaxSpeedMpS) and (acceleration > -1.0))
		{

			//T += 0.1;
			std::cout << "Recalculating LaneChange Path because Max_Vel of " << PathTest.Max_Vel << "exceeds Max velocity of " << MaxSpeedMpS << " mps. Dumping Path for debug " << std::endl;
			for (auto & lCPt : ChangeLanePath) std::cout << lCPt.X << ", " << lCPt.Y << ", " << lCPt.ThetaRads << std::endl;

			double vel_diff = PathTest.Max_Vel - MaxSpeedInLaneChangeMpS > 1.0 ? (PathTest.Max_Vel - MaxSpeedInLaneChangeMpS)*1.5 : 1.0;
			targetSpeed -= vel_diff;// (PathTest.Max_Vel - targetSpeed + 0.1);
			acceleration = safeAcceleration((targetSpeed - laststep_targetspeed) / T);
			if (acceleration < 0)
			{
				startspeed = laststep_targetspeed + acceleration * SimulatorRunloopPeriod;
				S_NowState = { ReferenceFPt.S, startspeed, 0.0 };
			}
			targetSpeed = laststep_targetspeed + (T * acceleration);
			displacement = laststep_targetspeed * T + 0.5 * acceleration * pow(T, 2.0);
			LaneChangeEndFPt.S = ReferenceFPt.S + displacement;
			S_EndState = { LaneChangeEndFPt.S , targetSpeed, 0.0 };
			Jerk = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
			std::cout << "targetspeed of " << targetSpeed << " calculated with Safe acceleration of " << acceleration << " giving displacement of " << displacement << "m." << std::endl;
			counter++;
			if (counter > 5)
			{
				ValidPath = true;
			}
			else
			{
				ChangeLanePath.clear();
			}
		}
		else
		{
			ValidPath = true;
		}

	}


	
	///////////////////////////////
	// Add path along lane going straight so lane change does not need to be recalcuated due to short path. 
	
	CartesianPoint LastCPt = ChangeLanePath.back();
	
	T = 1.2;
	S_NowState = { LaneChangeEndFPt.S, targetSpeed, 0.0 };
	D_NowState = { LaneChangeEndFPt.D, 0.0, 0.0 };
	S_EndState = { LaneChangeEndFPt.S + targetSpeed * (T), targetSpeed, 0.0 };
	D_EndState = { LaneChangeEndFPt.LaneCenterDCoord(targetLane), 0.0, 0.0 };
	JMT Jerk1 = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
	FPath.clear();
	for (double t = 0.0; t <= T/2.0 ; t += SimulatorRunloopPeriod)
	{
		//std::cout << "Adding Path, t = " << t << std::endl;
		FPt = Jerk1.JMTDisplacementAt(t);
		FPath.push_back(FPt);
	}
	std::vector<CartesianPoint> AddOnPath = map.ConvertCurveMaintainSpeed(FPath, LastCPt);
	CartesianPoint offset = { AddOnPath.at(0).X - LastCPt.X,
		AddOnPath.at(0).Y - LastCPt.Y };

	for (int i = 0; i<AddOnPath.size(); i++)
	{
		AddOnPath.at(i).X -= offset.X;
		AddOnPath.at(i).Y -= offset.Y;
	}

	ChangeLanePath.insert(ChangeLanePath.end(), AddOnPath.begin()+1, AddOnPath.end());
	///////////////////////////////

	//offset_path by any discrepencies from FrenetToCartesian from known location

	offset = { ChangeLanePath.at(0).X - ReferenceCPt.X,
		ChangeLanePath.at(0).Y - ReferenceCPt.Y };

	for (int i = 0; i<ChangeLanePath.size(); i++)
	{
		ChangeLanePath.at(i).X -= offset.X;
		ChangeLanePath.at(i).Y -= offset.Y;
	}

	_logger->info("Checking Final Lane Change Path.");
	Acc_Jerk PathTest = CheckPath(ChangeLanePath, SimulatorRunloopPeriod, input.SpeedMpS);
	_logger->info("Check Path finds Maximum Velocity in Path is {} ", PathTest.Max_Vel);
	_logger->info("Check Path finds Acceleration Maximum = {0} Fwd = {1} Lateral = {2}  ",
		PathTest.Max_Total_Acc, PathTest.Max_Fwd_Acc, PathTest.Max_Lat_Acc);
	_logger->info("Check Path finds Jerk Maximum = {0} Fwd = {1} Lateral = {2}  ",
		PathTest.Max_Total_Jerk, PathTest.Max_Fwd_Jerk, PathTest.Max_Lat_Jerk);
	_logger->flush();
	
	//double vel_diff = PathTest.Max_Vel - MaxSpeedMpS > 1.0 ? (PathTest.Max_Vel - MaxSpeedMpS)*1.25 : 1.0;
	
	if (input.Path.size() > 0)
		ChangeLanePath.insert(ChangeLanePath.begin(), input.Path.begin(), input.Path.end());

	for (auto & lCPt : ChangeLanePath) _logger->info("ChangeLane, {:03.4f}, {:03.4f}, {:03.4f}", lCPt.X, lCPt.Y, lCPt.ThetaRads);

	LastIterEndpointFPt = map.CartesianToFrenet(ChangeLanePath.back());
	return ChangeLanePath;
}

Acc_Jerk JMTBasedPlanner::CheckPath(std::vector<CartesianPoint> Path, double time_increment, double v_init, double a_init)
{
	//a_init and v_init have default values;


	Acc_Jerk Path_Characteristics;
	Path_Characteristics.Max_Vel = 0.0;
	Path_Characteristics.Max_Total_Acc = 0.0;
	Path_Characteristics.Max_Fwd_Acc = 0.0;
	Path_Characteristics.Max_Pos_Acc = 0.0;
	Path_Characteristics.Max_Neg_Acc = 0.0;
	Path_Characteristics.Max_Lat_Acc = 0.0;
	Path_Characteristics.Max_Total_Jerk = 0.0;
	Path_Characteristics.Max_Fwd_Jerk = 0.0;
	Path_Characteristics.Max_Lat_Jerk = 0.0;


	//measure instantaneous peak velocity , but acceleration over a longer period to get a smoother value.
	int StepsPerSegment = 10;
	double segment_distance = 0.0;
	double segment_acceleration;
	double delta_heading;
	double step_distance;
	double step_velocity;
	double last_step_velocity = v_init;
	double last_segment_velocity = v_init;
	double step_acceleration;
	double step_fwd_acc;
	double step_lat_acc;
	double last_step_acceleration = a_init;
	double step_lat_jerk;
	double step_fwd_jerk;
	double step_tot_jerk;


	for (int step = 1; step < Path.size(); step++)
	{
		delta_heading = headingdifference(Path.at(step).ThetaRads, Path.at(step - 1).ThetaRads);
		step_distance = Path.at(step).EuclidDistance(Path.at(step - 1));
		segment_distance += step_distance;
		step_velocity = step_distance / time_increment;
		Path_Characteristics.Max_Vel = Path_Characteristics.Max_Vel < step_velocity ? step_velocity : Path_Characteristics.Max_Vel;
		if (step % StepsPerSegment == 0)
		{
			double segment_velocity = segment_distance / (StepsPerSegment * time_increment);
			segment_acceleration = (segment_velocity - last_segment_velocity) / (StepsPerSegment * time_increment);
			Path_Characteristics.Max_Total_Acc = fabs(Path_Characteristics.Max_Total_Acc) < fabs(segment_acceleration) ? segment_acceleration : Path_Characteristics.Max_Total_Acc;

			segment_distance = 0;
			last_segment_velocity = segment_velocity;
		}
	}
	_logger->info("Max step Velocity = {}, Max Segment Acceleration = {}", Path_Characteristics.Max_Vel, Path_Characteristics.Max_Total_Acc);
	return(Path_Characteristics);
}