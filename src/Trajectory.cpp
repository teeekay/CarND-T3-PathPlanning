//
// Trajectory.cpp
// Created by Anthony M Knight 30/01/2018
//

#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip> //for printing roadmap2.
#include <algorithm>
#include <iterator>

#include "Trajectory.h"




double Trajectory::GetAcceleration()
{
	return Acceleration;
}

double Trajectory::GetSafeAcceleration(double accel)
{
	return ((accel >= 0) ? (accel> MaxFwdAccelerationMpSsq) ? MaxFwdAccelerationMpSsq : accel :
		(accel < MaxBrakingAccelerationMpSsq) ? MaxBrakingAccelerationMpSsq : accel);
}

void Trajectory::OffsetPath(std::vector<CartesianPoint> &CPath, CartesianPoint LastCPt)
{
	//offset_path by any discrepencies from FrenetToCartesian from known location
	CartesianPoint offset = { CPath.at(0).X - LastCPt.X, CPath.at(0).Y - LastCPt.Y };

	for (int i = 0; i < CPath.size(); i++)
	{
		CPath.at(i).X -= offset.X;
		CPath.at(i).Y -= offset.Y;
	}
}

//trim the path down to a new size where recalculated path can be added on 
//needed in case of decceleration or evasion
std::vector<CartesianPoint> Trajectory::TrimPath(std::vector<CartesianPoint> &Path, int NumberPoints = 10)
{

	std::vector<CartesianPoint> TrimmedPath;

	int pathsize = NumberPoints < Path.size() + 1 ? NumberPoints : Path.size();  //want to keep a buffer of existing pathpoints to account for any delays sending path (so there are still old pathpoints in front of car)
	if (pathsize > 1)
	{
		TrimmedPath.assign(Path.begin(), Path.begin() + pathsize - 1);
	}
	else
	{
		TrimmedPath.push_back(Path.back());
	}
	_logger2->info("Trimmed Path Size is {} with ({},{}) as endpoint", TrimmedPath.size(), TrimmedPath.back().X, TrimmedPath.back().Y);
	return TrimmedPath;
}


//Generate the Trajectory with the Jerk minimizing Algorithm
// GenerateZero is a flag to determine if to return point at time = 0.0 , or not
std::vector<FrenetPoint> Trajectory::GenerateJMTPath(FrenetPoint LastFPt, FrenetPoint DestFPt, FrenetPoint LastSpeed, FrenetPoint TargetSpeed, double T,
	bool GenerateZero, FrenetPoint LastAccel, FrenetPoint TargetAccel)
{
	double PathStartTime = GenerateZero? 0.0: SimulatorRunloopPeriod;

	std::vector<double> S_NowState = { LastFPt.S, LastSpeed.S, LastAccel.S };
	std::vector<double> D_NowState = { LastFPt.D, LastSpeed.D, LastAccel.D };

	std::vector<double> S_EndState = { DestFPt.S, TargetSpeed.S, TargetAccel.S };
	std::vector<double> D_EndState = { DestFPt.D, TargetSpeed.D, TargetAccel.D };

	JMT Jerk = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
	FrenetPoint lFPt;
	std::vector<FrenetPoint> FPath;
	
	for (double t = PathStartTime; t <= T; t += SimulatorRunloopPeriod)
	{
		lFPt = Jerk.JMTDisplacementAt(t);
		FPath.push_back(lFPt);
	}
	return FPath;
}

void Trajectory::LogCPath(std::vector<CartesianPoint> CPath)
{
	_logger2->warn("LogCPath dumping Cartesian Path with {} elements", CPath.size());
	for (CartesianPoint & lCPt : CPath)
	{	
		_logger2->warn("Cartesian = {:+3.3f} , {:+3.3f} , {:+3.3f} ", lCPt.X, lCPt.Y, lCPt.ThetaRads);
	}
}

void Trajectory::LogFPath(std::vector<FrenetPoint> FPath)
{
	_logger2->warn("LogFPath dumping Frenet Path with {} elements", FPath.size());
	//for (FrenetPoint & lFPt : FPath)
	for (auto it = std::begin(FPath); it != std::end(FPath); ++it)
	{
		_logger2->warn("Frenet = {:+3.3f}, {:+3.3f}", it->S, it->D);
	}
}

FrenetPoint Trajectory::GetFinalFAccel(std::vector<CartesianPoint>const & CPath) 
{
	double FSV1, FSV2, FDV1, FDV2;
	double CV2, CV1;
	FrenetPoint FAccel;
	FSV2 = (RangeS(map2.CartesianToFrenet(CPath.at(CPath.size() - 1)).S - map2.CartesianToFrenet(CPath.at(CPath.size() - 2)).S))/ SimulatorRunloopPeriod;
	FDV2 = (map2.CartesianToFrenet(CPath.at(CPath.size() - 1)).D - map2.CartesianToFrenet(CPath.at(CPath.size() - 2)).D)/ SimulatorRunloopPeriod;
	FSV1 = (RangeS(map2.CartesianToFrenet(CPath.at(CPath.size() - 2)).S - map2.CartesianToFrenet(CPath.at(CPath.size() - 3)).S))/ SimulatorRunloopPeriod;
	FDV1 = (map2.CartesianToFrenet(CPath.at(CPath.size() - 2)).D - map2.CartesianToFrenet(CPath.at(CPath.size() - 3)).D)/ SimulatorRunloopPeriod;
	FAccel = { (FSV2 - FSV1) / SimulatorRunloopPeriod, (FDV2 - FDV1) / SimulatorRunloopPeriod };
	_logger2->info("Trajectory: Calculated Frenet Acceleration of {:+2.3f}, {:+2.3f} at end of existing path.", FAccel.S, FAccel.D);
	CV2 = CPath.at(CPath.size() - 1).EuclidDistance(CPath.at(CPath.size() - 2))/ SimulatorRunloopPeriod;
	CV1 = CPath.at(CPath.size() - 2).EuclidDistance(CPath.at(CPath.size() - 3))/ SimulatorRunloopPeriod;
	FAccel = { (CV2 - CV1) / SimulatorRunloopPeriod, 0.0 };
	_logger2->info("Trajectory: Calculated Cartesian Acceleration of {:+2.3f}, {:+2.3f} at end of existing path.", FAccel.S, FAccel.D);

	return FAccel;
}


std::vector<CartesianPoint> Trajectory::InitiateTrajectory(PathPlannerInput input)
{
	FrenetPoint LastFPt = input.LocationFrenet;
	CartesianPoint LastCPt = input.LocationCartesian;
	_logger2->info("in InitiateTrajectory");

	double T = 1.0;
	double Displacement = 0.5 * MaxFwdAccelerationMpSsq * pow(T, 2.0);
	double TargetSpeed = MaxFwdAccelerationMpSsq * T;

	FrenetPoint StartSpeed = { 0.0, 0.0 };
	FrenetPoint LastSpeed = { TargetSpeed, 0.0 };
	FrenetPoint EndFPt = { LastFPt.S + Displacement, LastFPt.D };//move forward before moving sideways
	
	//generate frenet curve starting at the current location of the car
	std::vector<FrenetPoint> StartupFPath = GenerateJMTPath(LastFPt, EndFPt, StartSpeed, LastSpeed, T, true);
	// attempt to adjust for any effects of road curvature on speed while switching from frenet to cartesian
	std::vector<CartesianPoint> CPath = map2.ConvertCurveMaintainSpeed(StartupFPath, LastCPt);
	//adjust for any discrepancy between the known and calculated location of the path.
	OffsetPath(CPath, LastCPt);

	_logger2->info("Checking Initialization Path.");
	Acc_Jerk PathTest = CheckPath(CPath, SimulatorRunloopPeriod, input.SpeedMpS);

	LogCPath(CPath);
	return CPath;
}

//
// build a trajectory to remain in the lane and move to the center
// can decide to truncate existing path down to 10 points (useful for emergency braking or diversions,
// or just append to current path
std::vector<CartesianPoint> Trajectory::GenerateKeepInLaneTrajectory( PathPlannerInput input, double DesiredVelocity, bool Truncate)
{
	std::vector<FrenetPoint> FPath;
	std::vector<CartesianPoint> OutputPath, CPath;
	double Displacement, T;
	CartesianPoint EndCPt, PrevCPt;
	FrenetPoint EndFPt, NewEndFPt, EndFSpeed, TargetFSpeed, EndFAccel, TargetFAccel;

	_logger2->info("Trajectory: In GenerateKeepInLaneTrajectory: Truncate = {}.",Truncate);

	//Should only have to deal with situations where there is already a path,
	// but put code in place to deal with empty path - probably simulator blowup at that point anyway!
	if( input.Path.size() > 0)
	{
		OutputPath = Truncate ? TrimPath(input.Path) : input.Path;
		EndCPt = OutputPath.back();
		if (OutputPath.size() == 1)
		{
			PrevCPt = input.LocationCartesian;
		}
		else
		{
			PrevCPt = OutputPath.at(OutputPath.size() - 2);
		}
		EndFPt = Truncate ? map2.CartesianToFrenet(EndCPt): input.PathEndpointFrenet;
	}
	else
	{
		EndCPt = input.LocationCartesian;
		double SpeedMpS = fabs(input.SpeedMpS) < 0.1 ? 1.0 : input.SpeedMpS;
		PrevCPt = { input.LocationCartesian.X - SimulatorRunloopPeriod * SpeedMpS * cos(input.LocationCartesian.ThetaRads),
					input.LocationCartesian.Y - SimulatorRunloopPeriod * SpeedMpS * sin(input.LocationCartesian.ThetaRads) };
		EndFPt = input.LocationFrenet;
	}
	EndCPt.ThetaRads = atan2(EndCPt.Y - PrevCPt.Y, EndCPt.X - PrevCPt.X);
	
	//T is time to complete maneuver - make a longer Path if truncating existing path.
	T = Truncate? 2.0: 1.0;
	TargetFSpeed = { DesiredVelocity, 0.0 };
	
	if (OutputPath.size() == 0)
	{
		EndFSpeed.S = input.SpeedMpS;
	}else{
		EndFSpeed.S = (1 / SimulatorRunloopPeriod) * sqrt(pow(EndCPt.X - PrevCPt.X, 2.0) + pow(EndCPt.Y - PrevCPt.Y, 2.0));
		EndFSpeed.D = 0.0;  //need to fix these up to get proper S and D speeds
	}
	//calculate acceleration at end of path
	if (OutputPath.size() > 2)
	{
		EndFAccel = GetFinalFAccel(OutputPath);
	}
	else
	{
		EndFAccel = { 0.0, 0.0 };
	}
	
	Acceleration = GetSafeAcceleration((TargetFSpeed.S - EndFSpeed.S) / T);
	TargetFSpeed.S = EndFSpeed.S + (T * Acceleration);
	Displacement = EndFSpeed.S * T + 0.5 * Acceleration * pow(T, 2.0);
	if (TargetFSpeed.S < (DesiredVelocity * 0.95))
	{
		TargetFAccel = { Acceleration, 0.0 };
	}
	else
	{
		TargetFAccel = { 0.0, 0.0 };
	}

	NewEndFPt = { EndFPt.S + Displacement, EndFPt.LaneCenterDCoord(EndFPt.GetLane()) };

	_logger2->info("Trajectory: TargetFSpeed of {:3.3f} calculated from startspeed of {:3.3f} with Safe Acceleration of {:+3.3f} giving Displacement of {:2.3f} m. for StayInLanePath",
		TargetFSpeed.S, EndFSpeed.S, Acceleration, Displacement);

	bool ValidPath = false;
	int ReCalcCounter = 0;
	while (!ValidPath)
	{
		FPath.clear();
		FPath = GenerateJMTPath(EndFPt, NewEndFPt, EndFSpeed, TargetFSpeed, T, true, EndFAccel, TargetFAccel);
		CPath = map2.ConvertCurveMaintainSpeed(FPath, EndCPt);

		Acc_Jerk PathTest = CheckPath(CPath, SimulatorRunloopPeriod, EndFSpeed.S);
		_logger2->info("Check Path ReCalcCounter={ } finds Maximum Velocity ={:3.3f} while MaxSpeedMPS is {:3.3f} Acceleration Maximum = {:+3.3f} while design Acceleration was {:+2.3f} ",
			ReCalcCounter, PathTest.Max_Vel, MaxSpeedMpS, PathTest.Max_Total_Acc, Acceleration);
		
		if ((PathTest.Max_Vel > MaxSpeedMpS) and (Acceleration > -1.0))
		{
			ReCalcCounter++;
			// for now just set to zero.
			TargetFAccel = { 0.0, 0.0 };
			if (ReCalcCounter > 3)
			{
				_logger2->warn("Trajectory: Could not get path to meet specs, using last set.");
				ValidPath = true;  //give up. - maybe it will work!
			}
			else
			{

				_logger2->warn("Recalculating StayInLanePath because Max_Vel of {} exceeds Max velocity of {} mps. Dumping Path for debug:",
					PathTest.Max_Vel, MaxSpeedMpS);
				LogCPath(CPath);
				double vel_diff = PathTest.Max_Vel - MaxSpeedMpS > 1.0 ? (PathTest.Max_Vel - MaxSpeedMpS)*1.5 : 1.0;
				TargetFSpeed.S -= vel_diff;// (PathTest.Max_Vel - targetSpeed + 0.1);
				Acceleration = GetSafeAcceleration((TargetFSpeed.S - EndFSpeed.S) / T);
				TargetFSpeed.S = EndFSpeed.S + (T * Acceleration);
				Displacement = EndFSpeed.S * T + 0.5 * Acceleration * pow(T, 2.0);
				NewEndFPt.S = EndFPt.S + Displacement;
				_logger2->warn("TargetFSpeed of {:3.3f} calculated with Safe Acceleration of {:+3.3f} giving Displacement of {:3.3f} m.", TargetFSpeed.S, Acceleration, Displacement);
				CPath.clear();
			}
		}
		else if (PathTest.Max_Total_Acc < -5.0)
		{
			ReCalcCounter++;
			if (ReCalcCounter > 3)
			{
				ValidPath = true;
			}else{
				_logger2->warn("Recalculating StayInLanePath because Acceleration of {} exceeds Max Acceleration of {} mps^2. Dumping Path for debug:",
					PathTest.Max_Total_Acc, MaxBrakingAccelerationMpSsq);
				LogCPath(CPath); 
				Acceleration = MaxBrakingAccelerationMpSsq;
				TargetFSpeed.S = EndFSpeed.S + (T * Acceleration);
				if (TargetFSpeed.S < 0.0)
				{
					TargetFSpeed.S = 0.0;
					Acceleration = GetSafeAcceleration(EndFSpeed.S / 2.0 / T);
				}
				Displacement = EndFSpeed.S * T + 0.5 * Acceleration * pow(T, 2.0);
				NewEndFPt.S = EndFPt.S + Displacement;
				_logger2->warn("TargetFSpeed of {:3.3f} calculated with Safe Acceleration of {:+3.3f} giving Displacement of {:3.3f} m.", TargetFSpeed.S, Acceleration, Displacement);
				CPath.clear();
			}
		}else{
			ValidPath = true;
		}
	}
	OffsetPath(CPath, EndCPt);
	
	if (OutputPath.size() > 0)
	{
		OutputPath.insert(OutputPath.end(), CPath.begin()+1, CPath.end());
	}
	LogCPath(OutputPath);

	return OutputPath;
}

//
// build a trajectory to move 1 lane to the left or right
// can decide to truncate existing path down to 10 points (Lets change lanes now!),
// or just append to current path
std::vector<CartesianPoint> Trajectory::GenerateJMTLaneChangeTrajectory(PathPlannerInput input, int TargetLane, double DesiredVelocity, bool Truncate)
{
	std::vector<FrenetPoint> FPath;
	std::vector<CartesianPoint> OutputPath, CPath;
	double LaneChangeFactor,SDisplacement, Acceleration, T;
	CartesianPoint EndCPt, PrevCPt;
	int CurrentLane;
	FrenetPoint EndFPt, NewEndFPt, EndSpeed, TargetSpeed;
	
	assert(-1 < TargetLane < 3); // Make sure in range
	CurrentLane = input.LocationFrenet.GetLane();
	int LanesChange = TargetLane - CurrentLane;
	assert(abs(LanesChange) < 2);  // 1 lane at a time

	_logger2->info("In GenerateJMTLaneChangeTrajectory");

	//Should only have to deal with situations where there is already a path,
	// but put code in place to deal with empty path - probably simulator blowup at that point anyway!
	if (input.Path.size() > 0)
	{
		OutputPath = Truncate ? TrimPath(input.Path) : input.Path;
		EndCPt = OutputPath.back();
		if (OutputPath.size() == 1)
		{
			PrevCPt = input.LocationCartesian;
		}
		else
		{
			PrevCPt = OutputPath.at(OutputPath.size() - 2);
		}
		EndFPt = Truncate ? map2.CartesianToFrenet(EndCPt) : input.PathEndpointFrenet;
	}
	else
	{
		EndCPt = input.LocationCartesian;
		double SpeedMpS = fabs(input.SpeedMpS) < 0.1 ? 1.0 : input.SpeedMpS;
		PrevCPt = { input.LocationCartesian.X - SimulatorRunloopPeriod * SpeedMpS * cos(input.LocationCartesian.ThetaRads),
			input.LocationCartesian.Y - SimulatorRunloopPeriod * SpeedMpS * sin(input.LocationCartesian.ThetaRads) };
		EndFPt = input.LocationFrenet;
	}
	EndCPt.ThetaRads = atan2(EndCPt.Y - PrevCPt.Y, EndCPt.X - PrevCPt.X);

	//T is time to complete maneuver 
	T = 1.7;
	TargetSpeed = { DesiredVelocity, 0.0 };

	if (OutputPath.size() == 0)
	{
		EndSpeed.S = input.SpeedMpS;
	}
	else
	{
		EndSpeed.S = (1 / SimulatorRunloopPeriod) * sqrt(pow(EndCPt.X - PrevCPt.X, 2.0) + pow(EndCPt.Y - PrevCPt.Y, 2.0));
		EndSpeed.D = 0.0;  //need to fix these up to get proper S and D speeds
	}
	Acceleration = GetSafeAcceleration((TargetSpeed.S - EndSpeed.S) / T);
	TargetSpeed.S = EndSpeed.S + (T * Acceleration);
//	Displacement = EndFSpeed.S * T + 0.5 * Acceleration * pow(T, 2.0);
	LaneChangeFactor = ((EndSpeed.S + TargetSpeed.S) / 2.0 * T) / sqrt(pow(T*(TargetSpeed.S + EndSpeed.S) / 2.0, 2.0) + pow(3.9, 2.0));
	SDisplacement = (EndSpeed.S + TargetSpeed.S) / 2.0 * T * LaneChangeFactor;

	NewEndFPt = { EndFPt.S + SDisplacement, FrenetPoint::LaneCenterDCoord(TargetLane) };

	_logger2->info("TargetFSpeed of {:3.3f} calculated with Safe Acceleration of {:+3.3f} giving Displacement of {:2.3f} m. for LaneChangePath",
		TargetSpeed.S, Acceleration, SDisplacement);

	bool ValidPath = false;
	int ReCalcCounter = 0;
	while (!ValidPath)
	{
		FPath.clear();
		FPath = GenerateJMTPath(EndFPt, NewEndFPt, EndSpeed, TargetSpeed, T, true);
		CPath = map2.ConvertCurveMaintainSpeed(FPath, EndCPt);

		Acc_Jerk PathTest = CheckPath(CPath, SimulatorRunloopPeriod, EndSpeed.S);
		_logger2->info("Check Path ReCalcCounter={} finds Maximum Velocity ={:3.3f} while MaxSpeedMPS is {:3.3f} Acceleration Maximum = {:+3.3f} while design Acceleration was {:+2.3f} ",
			ReCalcCounter, PathTest.Max_Vel, MaxSpeedMpS, PathTest.Max_Total_Acc, Acceleration);

		if ((PathTest.Max_Vel > MaxSpeedMpS) and (Acceleration > -1.0))
		{
			ReCalcCounter++;
			if (ReCalcCounter > 3)
			{
				_logger2->warn("Could not get path to meet specs, using last set.");
				ValidPath = true;  //give up. - maybe it will work!
			}
			else
			{

				_logger2->warn("Recalculating StayInLanePath because Max_Vel of {} exceeds Max velocity of {} mps. Dumping Path for debug:",
					PathTest.Max_Vel, MaxSpeedMpS);
				LogCPath(CPath);
				double vel_diff = PathTest.Max_Vel - MaxSpeedMpS > 1.0 ? (PathTest.Max_Vel - MaxSpeedMpS)*1.5 : 1.0;
				TargetSpeed.S -= vel_diff;// (PathTest.Max_Vel - targetSpeed + 0.1);
				Acceleration = GetSafeAcceleration((TargetSpeed.S - EndSpeed.S) / T);
				TargetSpeed.S = EndSpeed.S + (T * Acceleration);
				LaneChangeFactor = ((EndSpeed.S + TargetSpeed.S) / 2.0 * T) / sqrt(pow(T*(TargetSpeed.S + EndSpeed.S) / 2.0, 2.0) + pow(3.9, 2.0));
				SDisplacement = (EndSpeed.S + TargetSpeed.S) / 2.0 * T * LaneChangeFactor; 
				NewEndFPt.S = EndFPt.S + SDisplacement;
				_logger2->warn("TargetFSpeed of {:3.3f} calculated with Safe Acceleration of {:+3.3f} giving Displacement of {:3.3f} m.", TargetSpeed.S, Acceleration, SDisplacement);
				CPath.clear();
			}
		}
		else if (PathTest.Max_Total_Acc < -5.0)
		{
			ReCalcCounter++;
			if (ReCalcCounter > 3)
			{
				ValidPath = true;
			}
			else
			{
				_logger2->warn("Recalculating StayInLanePath because Acceleration of {} exceeds Max Acceleration of {} mps^2. Dumping Path for debug:",
					PathTest.Max_Total_Acc, MaxBrakingAccelerationMpSsq);
				LogCPath(CPath);
				Acceleration = MaxBrakingAccelerationMpSsq;
				TargetSpeed.S = EndSpeed.S + (T * Acceleration);
				if (TargetSpeed.S < 0.0)
				{
					TargetSpeed.S = 0.1;
					Acceleration = GetSafeAcceleration((EndSpeed.S+TargetSpeed.S)/ 2.0 / T);
				}
				LaneChangeFactor = ((EndSpeed.S + TargetSpeed.S) / 2.0 * T) / sqrt(pow(T*(TargetSpeed.S + EndSpeed.S) / 2.0, 2.0) + pow(3.9, 2.0));
				SDisplacement = (EndSpeed.S + TargetSpeed.S) / 2.0 * T * LaneChangeFactor;
				//SDisplacement = EndFSpeed.S * T + 0.5 * Acceleration * pow(T, 2.0);
				NewEndFPt.S = EndFPt.S + SDisplacement;
				_logger2->warn("TargetFSpeed of {:3.3f} calculated with Safe Acceleration of {:+3.3f} giving Displacement of {:3.3f} m.", 
					TargetSpeed.S, Acceleration, SDisplacement);
				CPath.clear();
			}
		}
		else
		{
			ValidPath = true;
		}
	}
	
	///////////////////////////////
	// Add path along lane going straight so lane change does not need to be recalcuated due to short path. 

	T = 1.0;
	CartesianPoint LastCPt = CPath.back();
	FrenetPoint CurveEndFPt = FPath.back();
	FrenetPoint RunoutEndFPt = { CurveEndFPt.S + TargetSpeed.S * T, CurveEndFPt.D };
	FPath.clear();
	FPath = GenerateJMTPath(CurveEndFPt, RunoutEndFPt, TargetSpeed, TargetSpeed, T, true);
	std::vector<CartesianPoint> AddOnCPath = map2.ConvertCurveMaintainSpeed(FPath, LastCPt);
	OffsetPath(AddOnCPath, LastCPt);
	//only want 30 points down lane
	CPath.insert(CPath.end(), AddOnCPath.begin()+1, AddOnCPath.begin()+30);
	
	_logger2->info("Checking Final Lane Change Path.");
	Acc_Jerk PathTest = CheckPath(CPath, SimulatorRunloopPeriod, input.SpeedMpS);
	_logger2->info("Check Final LaneChange Path finds Maximum Velocity ={:3.3f} while MaxSpeedMPS is {:3.3f} Acceleration Maximum = {:+3.3f} while design Acceleration was {:+2.3f} ",
		 PathTest.Max_Vel, MaxSpeedMpS, PathTest.Max_Total_Acc, Acceleration);
	
	OffsetPath(CPath, EndCPt);
	OutputPath.insert(OutputPath.end(), CPath.begin()+1, CPath.end());
	
	LogCPath(OutputPath);
	
	return OutputPath;
}

Acc_Jerk Trajectory::CheckPath(std::vector<CartesianPoint> Path, double time_increment, double v_init, double a_init)
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
	_logger2->info("Max step Velocity = {}, Max Segment Acceleration = {}", Path_Characteristics.Max_Vel, Path_Characteristics.Max_Total_Acc);
	return(Path_Characteristics);
}