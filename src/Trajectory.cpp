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

FrenetPoint Trajectory::GetFrenetLocation(int pathsize)
{
	return BuiltPath.GetFrenetDescriptorsAt(BuiltPath.size() - (pathsize+1)).Displacement;
}

double Trajectory::GetAcceleration()
{
	return Acceleration;
}

double Trajectory::GetSafeAcceleration(double Accel)
{
	double NewAccel;
	if (Accel >= 0.0)
	{
		NewAccel = Accel > MaxFwdAccelerationMpSsq ? MaxFwdAccelerationMpSsq : Accel;
	}
	else
	{
		NewAccel = Accel < MaxBrakingAccelerationMpSsq ? MaxBrakingAccelerationMpSsq : Accel;
	}
	_TL->info("Acceleration adjusted from {:+3.3} to {:+3.3f}", Accel, NewAccel);
	
	return NewAccel;
}


//Generate the Trajectory with the Jerk minimizing Algorithm
// GenerateZero is a flag to determine if to return point at time = 0.0 , or not
std::vector<FrenetDescriptors> Trajectory::GenerateJMTDPath(FrenetPoint LastFPt, FrenetPoint DestFPt, FrenetPoint LastSpeed, FrenetPoint TargetSpeed, double T,
	bool GenerateZero, FrenetPoint LastAccel, FrenetPoint TargetAccel)
{
	double PathStartTime = GenerateZero ? 0.0 : SimulatorRunloopPeriod;

	std::vector<double> S_NowState = { LastFPt.S, LastSpeed.S, LastAccel.S };
	std::vector<double> D_NowState = { LastFPt.D, LastSpeed.D, LastAccel.D };

	std::vector<double> S_EndState = { DestFPt.S, TargetSpeed.S, TargetAccel.S };
	std::vector<double> D_EndState = { DestFPt.D, TargetSpeed.D, TargetAccel.D };

	JMT Jerk = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
	FrenetPoint lFPt;
	FrenetDescriptors lFDPt;
	std::vector<FrenetDescriptors> FPath;

	for (double t = PathStartTime; t <= T; t += SimulatorRunloopPeriod)
	{
		//lFPt = Jerk.JMTDisplacementAt(t);
		lFDPt = Jerk.JMTFrenetDescriptorsAt(t);
		FPath.push_back(lFDPt);
	}
	return FPath;
}


void Trajectory::LogCPath(std::vector<CartesianPoint> CPath)
{
	_TL->warn("LogCPath dumping Cartesian Path with {} elements", CPath.size());
	for (CartesianPoint & lCPt : CPath)
	{	
		_TL->warn("Cartesian = {:+3.3f} , {:+3.3f} , {:+3.3f} ", lCPt.X, lCPt.Y, lCPt.ThetaRads);
	}
}

void Trajectory::LogFPath(std::vector<FrenetPoint> FPath)
{
	_TL->warn("LogFPath dumping Frenet Path with {} elements", FPath.size());
	//for (FrenetPoint & lFPt : FPath)
	for (auto it = std::begin(FPath); it != std::end(FPath); ++it)
	{
		_TL->warn("Frenet = {:+3.3f}, {:+3.3f}", it->S, it->D);
	}
}


//
// build a trajectory to start the car moving straight down the road maintaining D
// to prevent rotation at low speed
//
std::vector<CartesianPoint> Trajectory::InitiateTrajectory(PathPlannerInput input)
{
	FrenetPoint LastFPt = input.LocationFrenet;
	CartesianPoint LastCPt = input.LocationCartesian;
	_TL->info("in InitiateTrajectory");
	_TL->flush();

	double T = 2.0;
	double Displacement = 0.5 * MaxFwdAccelerationMpSsq * pow(T, 2.0);
	double TargetSpeed = MaxFwdAccelerationMpSsq * T;

	FrenetPoint StartSpeed = { 0.0, 0.0 };
	FrenetPoint LastSpeed = { TargetSpeed, 0.0 };
	FrenetPoint EndFPt = { LastFPt.S + Displacement, LastFPt.D };

	//generate frenet curve starting at the current location of the car
	std::vector<FrenetDescriptors> StartupFDPath = GenerateJMTDPath(LastFPt, EndFPt, StartSpeed, LastSpeed, T, true);

	PathPoint lPPt;
	int dequesize;
	for (FrenetDescriptors lFDPt : StartupFDPath)
	{
		lPPt.FDPt = lFDPt;
		lPPt.CPt = map2.FrenetToCartesian(lFDPt.Displacement);
		dequesize = BuiltPath.AddPathPoint(lPPt);
	}
	std::vector<CartesianPoint> CPath = BuiltPath.GetCPath();

	_TL->info("Checking Initialization Path with size of {}.", dequesize);
	_TL->flush( );
	Acc_Jerk PathTest = CheckPath(CPath, SimulatorRunloopPeriod, input.SpeedMpS);
	BuiltPath.logpath( );
 	return CPath;
}


//
// build a trajectory to remain in the lane and move to the center
// can decide to truncate existing path down to 10 points (useful for emergency braking or diversions,
// or just append to current path
std::vector<CartesianPoint> Trajectory::GenerateKeepInLaneTrajectory(PathPlannerInput input, double DesiredVelocity, bool Truncate,
	double FinalFPtDOffset )
{
	int SafeToTrimTo = 5;
	//Truncate = true;
	//write out first points in input.path -here
	std::vector<FrenetPoint> FPath;
	std::vector<CartesianPoint> OutputPath, CPath;
	double Displacement, T;
	CartesianPoint EndCPt, PrevCPt, Offset;
	FrenetPoint EndFPt, NewEndFPt, EndFSpeed, TargetFSpeed, EndFAccel, TargetFAccel;

	int PathSize = input.Path.size();
	int DequeSize = BuiltPath.size();
	DequeSize = BuiltPath.TrimPathDequeAtStart(DequeSize - PathSize);

	if (Truncate)
	{
		DequeSize = BuiltPath.TrimPathDequeAtEnd(DequeSize - (SafeToTrimTo+1));  //cut to 5 points plus the current location
	}

	FrenetDescriptors lFDPt = BuiltPath.GetFrenetDescriptorsAt(DequeSize - 1);
	FrenetDescriptors lFDPt1 = BuiltPath.GetFrenetDescriptorsAt(DequeSize - 2);
	EndFPt = lFDPt.Displacement;
//	EndFSpeed = lFDPt.Velocity;
	EndFSpeed = {(lFDPt.Displacement.S - lFDPt1.Displacement.S) / 0.02, (lFDPt.Displacement.D - lFDPt1.Displacement.D) / 0.02 };
	EndFAccel = lFDPt.Acceleration;

	T = 1.0;
	TargetFSpeed = { DesiredVelocity, 0.0 };

	Acceleration = GetSafeAcceleration((TargetFSpeed.S - EndFSpeed.S) / T);
	TargetFSpeed.S = EndFSpeed.S + (T * Acceleration);
	Displacement = EndFSpeed.S * T + 0.5 * Acceleration * pow(T, 2.0);
	if (TargetFSpeed.S < (DesiredVelocity * 0.90))
	{
		TargetFAccel = { Acceleration, 0.0 };
	}
	else
	{
		TargetFAccel = { 0.0, 0.0 };
	}

	NewEndFPt = { EndFPt.S + Displacement, EndFPt.LaneCenterDCoord(EndFPt.GetLane())+FinalFPtDOffset };

	_TL->info("Trajectory: TargetFSpeed of {:3.3f} calculated from startspeed of {:3.3f} with Safe Acceleration of {:+3.3f} giving Displacement of {:2.3f} m. for StayInLanePath",
		TargetFSpeed.S, EndFSpeed.S, Acceleration, Displacement);

	std::vector<FrenetDescriptors> FDPath;
	bool ValidPath = false;
	int ReCalcCounter = 0;
	while (!ValidPath)
	{
		FDPath.clear();
		if (ReCalcCounter > 0 )
		{
			if (Truncate)
			{
				DequeSize = BuiltPath.TrimPathDequeAtEnd(DequeSize - (SafeToTrimTo + 1));  //cut to 10 points plus the current location
			}
			else
			{
				DequeSize = BuiltPath.TrimPathDequeAtEnd(DequeSize - PathSize);
			}
		}
		_TL->info("Traj: startpoint, {}, {} V.S= {}, A.S = {} endpoint: {}, {} V.S= {},  A.S = {} , T= {}.",
			EndFPt.S, EndFPt.D, EndFSpeed.S, EndFAccel.S,
			NewEndFPt.S, NewEndFPt.D, TargetFSpeed.S, TargetFAccel.S, T);
		_TL->flush();

		FDPath = GenerateJMTDPath(EndFPt, NewEndFPt, EndFSpeed, TargetFSpeed, T, false, EndFAccel, TargetFAccel);
		PathPoint lPPt;
		std::vector<CartesianPoint> lCPts;
		for (FrenetDescriptors lFDPt : FDPath)
		{
			lPPt.FDPt = lFDPt;
			lPPt.CPt = map2.FrenetToCartesian(lFDPt.Displacement);
			DequeSize = BuiltPath.AddPathPoint(lPPt);
		}
		CPath = BuiltPath.GetCPath();

		Acc_Jerk PathTest = CheckPath(CPath, SimulatorRunloopPeriod, EndFSpeed.S);
		_TL->info("Check Path ReCalcCounter={} finds Maximum Velocity ={:3.3f} while MaxSpeedMPS is {:3.3f} Acceleration Maximum = {:+3.3f} while design Acceleration was {:+2.3f} ",
			ReCalcCounter, PathTest.Max_Vel, MaxSpeedMpS, PathTest.Max_Total_Acc, Acceleration);

		if ((PathTest.Max_Vel > MaxSpeedMpS) and (Acceleration > -1.0))
		{
			ReCalcCounter++;
			// for now just set to zero.
			TargetFAccel = { 0.0, 0.0 };
			if (ReCalcCounter > 3)
			{
				_TL->warn("Trajectory: Could not get path to meet specs, using last set.");
				ValidPath = true;  //give up. - maybe it will work!
			}
			else
			{
				_TL->warn("Recalculating StayInLanePath because Max_Vel of {} exceeds Max velocity of {} mps. Dumping Path for debug:",
					PathTest.Max_Vel, MaxSpeedMpS);
				BuiltPath.logpath( );//	LogCPath(CPath);
				double vel_diff = PathTest.Max_Vel - MaxSpeedMpS > 1.0 ? (PathTest.Max_Vel - MaxSpeedMpS)*1.5 : 1.0;
				TargetFSpeed.S -= vel_diff;// (PathTest.Max_Vel - targetSpeed + 0.1);
				Acceleration = GetSafeAcceleration((TargetFSpeed.S - EndFSpeed.S) / T);
				TargetFSpeed.S = EndFSpeed.S + (T * Acceleration);
				Displacement = EndFSpeed.S * T + 0.5 * Acceleration * pow(T, 2.0);
				NewEndFPt.S = EndFPt.S + Displacement;
				_TL->warn("TargetFSpeed of {:3.3f} calculated with Safe Acceleration of {:+3.3f} giving Displacement of {:3.3f} m.", TargetFSpeed.S, Acceleration, Displacement);
				CPath.clear();
			}
		}
		else
		{
			ValidPath = true;
		}
	}

	
	BuiltPath.logpath( );
	return CPath;
}


//
// build a trajectory to move 1 lane to the left or right
// can decide to truncate existing path down to 10 points (Lets change lanes now!),
// or just append to current path
std::vector<CartesianPoint> Trajectory::GenerateJMTLaneChangeTrajectory(PathPlannerInput input, int TargetLane, double DesiredVelocity, bool Truncate)
{
	int SafeToTrimTo = 5;
	std::vector<FrenetPoint> FPath, FPathRunout;
	std::vector<CartesianPoint> OutputPath, CPath;
	double LaneChangeFactor, SDisplacement, Acceleration, T;
	CartesianPoint EndCPt, PrevCPt;
	int CurrentLane;
	FrenetPoint EndFPt, NewEndFPt, EndFSpeed, TargetFSpeed, EndFAccel, TargetFAccel;
	
	assert(-1 < TargetLane < 3); // Make sure in range
	CurrentLane = input.LocationFrenet.GetLane();
	int LanesChange = TargetLane - CurrentLane;
	assert(abs(LanesChange) < 2);  // 1 lane at a time

	_TL->info("In GenerateJMTLaneChangeTrajectory");

	int PathSize = input.Path.size();
	int DequeSize = BuiltPath.size();
	DequeSize = BuiltPath.TrimPathDequeAtStart(DequeSize - PathSize);

	if (Truncate)
	{
		DequeSize = BuiltPath.TrimPathDequeAtEnd(DequeSize - (SafeToTrimTo+1));  //cut to 5 points plus the current location
	}

	T = 2.0;
	FrenetDescriptors lFDPt = BuiltPath.GetFrenetDescriptorsAt(DequeSize - 1);
	FrenetDescriptors lFDPt1 = BuiltPath.GetFrenetDescriptorsAt(DequeSize - 2);
	EndFPt = lFDPt.Displacement;
	EndFSpeed = { (lFDPt.Displacement.S - lFDPt1.Displacement.S) / 0.02, (lFDPt.Displacement.D - lFDPt1.Displacement.D) / 0.02 };
	EndFAccel = lFDPt.Acceleration;

	TargetFSpeed = { DesiredVelocity, 0.0 };

	Acceleration = GetSafeAcceleration((TargetFSpeed.S - EndFSpeed.S) / T);
	TargetFSpeed.S = EndFSpeed.S + (T * Acceleration);
	LaneChangeFactor = ((EndFSpeed.S + TargetFSpeed.S) / 2.0 * T) / sqrt(pow(T*(TargetFSpeed.S + EndFSpeed.S) / 2.0, 2.0) + pow(3.9, 2.0));
	SDisplacement = (EndFSpeed.S + TargetFSpeed.S) / 2.0 * T * LaneChangeFactor;

	if (TargetFSpeed.S < DesiredVelocity and abs(TargetFSpeed.S - DesiredVelocity)/DesiredVelocity > 0.1 )
	{
		TargetFAccel = { Acceleration, 0.0 };
	}
	else
	{
		TargetFAccel = { 0.0, 0.0 };
	}

	NewEndFPt = { EndFPt.S + SDisplacement, FrenetPoint::LaneCenterDCoord(TargetLane) };

	_TL->info("Trajectory: TargetFSpeed of {:3.3f} calculated from startspeed of {:3.3f} with Safe Acceleration of {:+3.3f} giving Displacement of {:2.3f} m. for LaneChange Trajectory",
		TargetFSpeed.S, EndFSpeed.S, Acceleration, SDisplacement);

	
	std::vector<FrenetDescriptors> FDPath;

	bool ValidPath = false;
	int ReCalcCounter = 0;
	while (!ValidPath)
	{
		FDPath.clear();
		if (ReCalcCounter > 0)
		{
			if (Truncate)
			{
				DequeSize = BuiltPath.TrimPathDequeAtEnd(DequeSize - (SafeToTrimTo+1));  //cut to 5 points plus the current location
			}
			else
			{
				DequeSize = BuiltPath.TrimPathDequeAtEnd(DequeSize - PathSize);
			}
		}

		_TL->info("Traj: startpoint, {}, {} V.S= {}, A.S = {} endpoint: {}, {} V.S= {},  A.S = {} , T= {}.",
			EndFPt.S, EndFPt.D, EndFSpeed.S, EndFAccel.S,
			NewEndFPt.S, NewEndFPt.D, TargetFSpeed.S, TargetFAccel.S, T);

		FDPath = GenerateJMTDPath(EndFPt, NewEndFPt, EndFSpeed, TargetFSpeed, T, false, EndFAccel, TargetFAccel);
		PathPoint lPPt;
		std::vector<CartesianPoint> lCPts;
		for (FrenetDescriptors lFDPt : FDPath)
		{
			lPPt.FDPt = lFDPt;
			lPPt.CPt = map2.FrenetToCartesian(lFDPt.Displacement);
			DequeSize = BuiltPath.AddPathPoint(lPPt);
		}
		CPath = BuiltPath.GetCPath();

		Acc_Jerk PathTest = CheckPath(CPath, SimulatorRunloopPeriod, EndFSpeed.S);
		_TL->info("Check Path ReCalcCounter={} finds Maximum Velocity ={:3.3f} while MaxSpeedMPS is {:3.3f} Acceleration Maximum = {:+3.3f} while design Acceleration was {:+2.3f} ",
			ReCalcCounter, PathTest.Max_Vel, MaxSpeedMpS, PathTest.Max_Total_Acc, Acceleration);

		if ((PathTest.Max_Vel > MaxSpeedMpS) and (Acceleration > -1.0))
		{
			// for now just set to zero.
			TargetFAccel = { 0.0, 0.0 };
			if (ReCalcCounter > 3)
			{
				_TL->warn("Trajectory: Could not get path to meet specs, using last set.");
				ValidPath = true;  //give up. - maybe it will work!
			}
			else
			{
				ReCalcCounter++;
				_TL->warn("Recalculating Change Lane Trajectory because Max_Vel of {} exceeds Max velocity of {} mps. Dumping Path for debug:",
					PathTest.Max_Vel, MaxSpeedMpS);
				BuiltPath.logpath( ); // LogCPath(CPath);
				double vel_diff = PathTest.Max_Vel - MaxSpeedMpS > 1.0 ? (PathTest.Max_Vel - MaxSpeedMpS)*1.5 : 1.0;
				TargetFSpeed.S -= vel_diff;// (PathTest.Max_Vel - targetSpeed + 0.1);
				Acceleration = GetSafeAcceleration((TargetFSpeed.S - EndFSpeed.S) / T);
				TargetFSpeed.S = EndFSpeed.S + (T * Acceleration);
				LaneChangeFactor = ((EndFSpeed.S + TargetFSpeed.S) / 2.0 * T) / sqrt(pow(T*(TargetFSpeed.S + EndFSpeed.S) / 2.0, 2.0) + pow(3.9, 2.0));
				SDisplacement = (EndFSpeed.S + TargetFSpeed.S) / 2.0 * T * LaneChangeFactor; 
				NewEndFPt.S = EndFPt.S + SDisplacement;
				_TL->warn("TargetFSpeed of {:3.3f} calculated with Safe Acceleration of {:+3.3f} giving Displacement of {:3.3f} m.", TargetFSpeed.S, Acceleration, SDisplacement);
				CPath.clear();
			}
		}
		else
		{
			ValidPath = true;
		}
	}
	

	BuiltPath.logpath( );
	return CPath;
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
	_TL->info("Max step Velocity = {}, Max Segment Acceleration = {}", Path_Characteristics.Max_Vel, Path_Characteristics.Max_Total_Acc);
	return(Path_Characteristics);
}

//Generate the Trajectory with the Jerk minimizing Algorithm
// GenerateZero is a flag to determine if to return point at time = 0.0 , or not
//std::vector<FrenetPoint> Trajectory::GenerateJMTPath(FrenetPoint LastFPt, FrenetPoint DestFPt, FrenetPoint LastSpeed, FrenetPoint TargetSpeed, double T,
//	bool GenerateZero, FrenetPoint LastAccel, FrenetPoint TargetAccel)
//{
//	double PathStartTime = GenerateZero ? 0.0 : SimulatorRunloopPeriod;
//
//	std::vector<double> S_NowState = { LastFPt.S, LastSpeed.S, LastAccel.S };
//	std::vector<double> D_NowState = { LastFPt.D, LastSpeed.D, LastAccel.D };
//
//	std::vector<double> S_EndState = { DestFPt.S, TargetSpeed.S, TargetAccel.S };
//	std::vector<double> D_EndState = { DestFPt.D, TargetSpeed.D, TargetAccel.D };
//
//	JMT Jerk = JMT(S_NowState, D_NowState, S_EndState, D_EndState, T);
//	FrenetPoint lFPt;
//	FrenetDescriptors lFDPt;
//	std::vector<FrenetPoint> FPath;
//
//	for (double t = PathStartTime; t <= T; t += SimulatorRunloopPeriod)
//	{
//		//lFPt = Jerk.JMTDisplacementAt(t);
//		lFDPt = Jerk.JMTFrenetDescriptorsAt(t);
//		FPath.push_back(lFDPt.Displacement);
//	}
//	return FPath;
//}
//
//void Trajectory::OffsetPath(std::vector<CartesianPoint> &CPath, CartesianPoint LastCPt)
//{
//	//offset_path by any discrepencies from FrenetToCartesian from known location
//	CartesianPoint offset = { CPath.at(0).X - LastCPt.X, CPath.at(0).Y - LastCPt.Y };
//
//	for (int i = 0; i < CPath.size( ); i++)
//	{
//		CPath.at(i).X -= offset.X;
//		CPath.at(i).Y -= offset.Y;
//	}
//}
//
//CartesianPoint Trajectory::GetOffset(CartesianPoint CalcCPt, CartesianPoint KnownCPt)
//{
//	//offset_path by any discrepencies from FrenetToCartesian from known location
//	CartesianPoint Offset = { CalcCPt.X - KnownCPt.X, CalcCPt.Y - KnownCPt.Y };
//	return Offset;
//}
//
////trim the path down to a new size where recalculated path can be added on 
////needed in case of decceleration or evasion
//std::vector<CartesianPoint> Trajectory::TrimPath(std::vector<CartesianPoint> &Path, int NumberPoints = 10)
//{
//
//	std::vector<CartesianPoint> TrimmedPath;
//
//	int pathsize = NumberPoints < Path.size( ) + 1 ? NumberPoints : Path.size( );  //want to keep a buffer of existing pathpoints to account for any delays sending path (so there are still old pathpoints in front of car)
//	if (pathsize > 1)
//	{
//		TrimmedPath.assign(Path.begin( ), Path.begin( ) + pathsize - 1);
//	}
//	else
//	{
//		TrimmedPath.push_back(Path.back( ));
//	}
//	_TL->info("Trimmed Path Size is {} with ({},{}) as endpoint", TrimmedPath.size( ), TrimmedPath.back( ).X, TrimmedPath.back( ).Y);
//	return TrimmedPath;
//}
//
//FrenetPoint Trajectory::GetFinalFAccel(std::vector<CartesianPoint>const & CPath)
//{
//	double FSV1, FSV2, FDV1, FDV2;
//	double CV2, CV1;
//	FrenetPoint FAccel;
//	int AvgLength = CPath.size( ) > 10 ? 5 : CPath.size( ) / 2;
//	int CPathEndPtr = CPath.size( ) - 1;
//	double DelT = double(AvgLength)*SimulatorRunloopPeriod;
//	FSV2 = (RangeS(map2.CartesianToFrenet(CPath.at(CPathEndPtr)).S - map2.CartesianToFrenet(CPath.at(CPathEndPtr - AvgLength)).S)) / DelT;
//	FDV2 = (map2.CartesianToFrenet(CPath.at(CPathEndPtr)).D - map2.CartesianToFrenet(CPath.at(CPathEndPtr - AvgLength)).D) / DelT;
//	FSV1 = (RangeS(map2.CartesianToFrenet(CPath.at(CPathEndPtr - AvgLength)).S - map2.CartesianToFrenet(CPath.at(CPathEndPtr - AvgLength * 2)).S)) / DelT;
//	FDV1 = (map2.CartesianToFrenet(CPath.at(CPathEndPtr - AvgLength)).D - map2.CartesianToFrenet(CPath.at(CPathEndPtr - AvgLength * 2)).D) / DelT;
//	FAccel = { (FSV2 - FSV1) / DelT, (FDV2 - FDV1) / DelT };
//	_TL->info("Trajectory: Frenet Acceleration calculated with DelT= {:+2.3f} gives {:+2.3f}, {:+2.3f} at end of existing path.", DelT, FAccel.S, FAccel.D);
//	CV2 = CPath.at(CPathEndPtr).EuclidDistance(CPath.at(CPathEndPtr - AvgLength)) / DelT;
//	CV1 = CPath.at(CPathEndPtr - AvgLength).EuclidDistance(CPath.at(CPathEndPtr - AvgLength * 2)) / DelT;
//	FAccel = { (CV2 - CV1) / DelT, 0.0 };
//	_TL->info("Trajectory: Cartesian Acceleration calculated with DelT= {:+2.3f} gives {:+2.3f}, {:+2.3f} at end of existing path.", DelT, FAccel.S, FAccel.D);
//
//	return FAccel;
//}

//std::vector<CartesianPoint> Trajectory::RecalcTrajectory(PathPlannerInput input, double DOffset)
//{
//	int PathSize = input.Path.size( );
//	int DequeSize = BuiltPath.size( );
//	DequeSize = BuiltPath.TrimPathDequeAtStart(DequeSize - PathSize);
//	std::vector<CartesianPoint> lCPts, CPath;
//	CPath = BuiltPath.GetCPath( );
//	_TL->info("Traj: Rebuild path - first print out old deque path.");
//	LogCPath(CPath);
//	CPath.clear( );
//
//	double T = double(BuiltPath.size( ))*0.02;
//	FrenetDescriptors lFEndDPt = BuiltPath.GetFrenetDescriptorsAt(DequeSize - 1);
//	FrenetDescriptors lFEndDPt1 = BuiltPath.GetFrenetDescriptorsAt(DequeSize - 2);
//
//	FrenetDescriptors lFStartDPt = BuiltPath.GetFrenetDescriptorsAt(1);
//	FrenetDescriptors lFStartDPt1 = BuiltPath.GetFrenetDescriptorsAt(0);
//
//	lFEndDPt.Velocity = { (lFEndDPt.Displacement.S - lFEndDPt1.Displacement.S) / 0.02,
//		(lFEndDPt.Displacement.D - lFEndDPt1.Displacement.D) / 0.02 };
//	lFStartDPt1.Velocity = { (lFStartDPt.Displacement.S - lFStartDPt1.Displacement.S) / 0.02,
//		(lFStartDPt.Displacement.D - lFStartDPt1.Displacement.D) / 0.02 };
//
//	DequeSize = BuiltPath.TrimPathDequeAtEnd(BuiltPath.size( ) - 1);  //cut to 1 point plus the current location
//
//	lFEndDPt.Displacement.D = lFEndDPt.Displacement.D - DOffset;
//
//	_TL->info("Traj: startpoint, {}, {} V.S= {}, A.S = {} endpoint: {}, {} V.S= {},  A.S = {} , T= {}.",
//		lFStartDPt.Displacement.S, lFStartDPt.Displacement.D, lFStartDPt.Velocity.S, lFStartDPt1.Acceleration.S,
//		lFEndDPt.Displacement.S, lFEndDPt.Displacement.D, lFEndDPt.Velocity.S, lFEndDPt.Acceleration.S, T);
//
//	std::vector<FrenetDescriptors> FDPath = GenerateJMTDPath(
//		lFStartDPt1.Displacement, lFEndDPt.Displacement,
//		lFStartDPt1.Velocity, lFEndDPt.Velocity,
//		T, false,
//		lFStartDPt1.Acceleration, lFEndDPt.Acceleration);
//
//	PathPoint lPPt;
//	for (FrenetDescriptors lFDPt : FDPath)
//	{
//		lPPt.FDPt = lFDPt;
//		lPPt.CPt = map2.FrenetToCartesian(lFDPt.Displacement);
//		DequeSize = BuiltPath.AddPathPoint(lPPt);
//	}
//	CPath = BuiltPath.GetCPath( );
//	_TL->info("Traj: Rebuild path - print out new deque path.");
//	LogCPath(CPath);
//	return CPath;
//}
