//
// RoadMap.cpp
// Created by Anthony M Knight 25/01/2018
//

#include <vector>
#include <iostream>
#include <iomanip> //for printing roadmap.
#include <algorithm>
#include <iterator>

int TargetLane;
int NumLanes = 3;
double TimeIncrement = 0.1;
int NumIncrements = 20; // move from 30

#include "RoadMap.h"

int FwdLimit = 120;
int RevLimit = 80;


// set up to create a grid which is divided into lanes, and 1 m increments.
// looking forward 120 m (> 5 seconds at 50 mph)
// and backwards 80 m


int RoadMap::CreateRoadMap() {

	double EgoPosition = double(RevLimit);
	EgoSpeedMpS = (ThisStepInput.SpeedMpS < 1.0) ? 2.0 : ThisStepInput.SpeedMpS;

	//std::vector< std::vector <int>>RoadMapNow(NumLanes, std::vector<int>(FwdLimit + RevLimit, 0)); // lane;
	std::vector<std::vector<std::vector< std::vector <int>>>> RoadMaps(2, std::vector<std::vector<std::vector<int>>>
		(NumIncrements, std::vector<std::vector<int>>(NumLanes, std::vector<int>(FwdLimit + RevLimit, 0))));

	for (int Interval = 0; Interval < NumIncrements; Interval++)
	{
		int EgoNow = floor(EgoPosition + (EgoSpeedMpS * Interval * TimeIncrement));
		RoadMaps[EgoCarLayer][Interval][EgoCar.GetLane()][EgoNow] = -1;  // Assign Ego Car ID -1 on Plan
	}

	int localS;
	for (auto& otherCar : ThisStepInput.OtherCars)
	{
		for (int Interval = 0; Interval < NumIncrements; Interval++)
		{
			//assume that cars are remaining in lane for this approximation
			//Todo improve to finding lane 
			localS = floor((otherCar.LocationFrenet.S + otherCar.Speed2DMagnitudeMpS()*Interval*TimeIncrement - EgoCar.S + RevLimit));
			if (localS > -1 and localS < (FwdLimit + RevLimit))
				RoadMaps[OtherCarsLayer][Interval][otherCar.LocationFrenet.GetLane()][localS] = otherCar.id + 1;// need to offset so car 0 doesn't dispppear!
		}
	}
//#define PRINTPLAN 1
#ifdef PRINTPLAN
//	for (int t = 0; t < NumIncrements; t += 10)
	for (int t = 0; t < 1; t += 10)
	{
		std::cout << "At t = " << double(t) * TimeIncrement << "." << std::endl;
		std::cout << std::hex;
		for (int i = 0; i < NumLanes; i++) {
			for (int j = 0; j < (FwdLimit + RevLimit); j++) {
				if (RoadMaps[OtherCarsLayer][t][i][j] == 0 and RoadMaps[EgoCarLayer][t][i][j] == 0)
				{
					std::cout << "-";
				}
				else if (RoadMaps[OtherCarsLayer][t][i][j] == 0 and RoadMaps[EgoCarLayer][t][i][j] == -1)
				{
						std::cout << "E";
				}
				else if (RoadMaps[OtherCarsLayer][t][i][j] != 0 and RoadMaps[EgoCarLayer][t][i][j] == -1)
				{
					std::cout << "*";
				}
				else
				{
					std::cout << RoadMaps[OtherCarsLayer][t][i][j];
				}
			}
			std::cout << std::endl;
		}
		std::cout << std::dec;
	}
#endif
	RoadMapDeck = RoadMaps;
	check_lanes();
	return 0;
}


int RoadMap::check_lanes() {
	std::vector<std::vector< std::vector <int>>> RoadMapNow = RoadMapDeck[OtherCarsLayer];
	
	std::vector <int>temp(NumLanes, 0);
	
	
	for (int t = 0; t < NumIncrements; t++) {
		clearlanelengthsFWD.push_back(temp);
		clearlanelengthsBACK.push_back(temp);
		int EgoOffset = floor(EgoSpeedMpS*t*TimeIncrement);

		for (int lane = 0; lane < NumLanes; lane++) {
			auto itFWD = std::find_if(RoadMapNow.at(t).at(lane).begin() + RevLimit + EgoOffset, RoadMapNow.at(t).at(lane).end(), [](int x) { return x > 0; });
			auto itBACK = std::find_if(RoadMapNow.at(t).at(lane).rbegin() + (FwdLimit - EgoOffset - 1), RoadMapNow.at(t).at(lane).rend(), [](int x) { return x > 0; });
			clearlanelengthsFWD.at(t).at(lane) = std::distance(RoadMapNow.at(t).at(lane).begin() + RevLimit + EgoOffset, itFWD);
			clearlanelengthsBACK.at(t).at(lane) = -(std::distance(itBACK, RoadMapNow.at(t).at(lane).rbegin() + (FwdLimit - EgoOffset - 1)));
		}

	}
	
	MinLaneClearFWD.clear();
	MinLaneClearBACK.clear();

	for (int lane = 0; lane < NumLanes; lane++) {
		int minclearfwd = 120;
		int minclearback = 80;
		for (int t = 0; t < NumIncrements; t++) {
			if (minclearfwd > clearlanelengthsFWD.at(t).at(lane)) minclearfwd = clearlanelengthsFWD.at(t).at(lane);
			if (minclearback > clearlanelengthsBACK.at(t).at(lane)) minclearback = clearlanelengthsBACK.at(t).at(lane);
		}
		MinLaneClearFWD.push_back(minclearfwd);
		MinLaneClearBACK.push_back(minclearback);
	}

	_loggerRoadMap->info("RoadMap: Next {:1.2f} s clearances FWD [ {:d}, {:d}, {:d} ] , BACK [ {:d}, {:d}, {:d} ]. currently FWD [ {:d}, {:d}, {:d} ] , BACK [ {:d}, {:d}, {:d} ]", 
	        double(NumIncrements)*TimeIncrement,
		    MinLaneClearFWD.at(0), MinLaneClearFWD.at(1), MinLaneClearFWD.at(2),
		    MinLaneClearBACK.at(0), MinLaneClearBACK.at(1), MinLaneClearBACK.at(2),
			clearlanelengthsFWD.at(0).at(0), clearlanelengthsFWD.at(0).at(1), clearlanelengthsFWD.at(0).at(2),
		    clearlanelengthsBACK.at(0).at(0), clearlanelengthsBACK.at(0).at(1), clearlanelengthsBACK.at(0).at(2));
	return 0;
}



int RoadMap::CheckForLaneChange() {

	int targetLane = -1;

	//if (clearlanelengthsFWD.at(0).at(EgoCar.GetLane()) < 50) {  //don't check for lanechange if still > 50 m clear ahead for next couple seconds
	if ( MinLaneClearFWD.at(EgoCar.GetLane())< 60) {
		//std::cout << "Now Fwd [" << clearlanelengthsFWD.at(0).at(0) <<", " << clearlanelengthsFWD.at(0).at(1) <<", " << clearlanelengthsFWD.at(0).at(2) << "] ";
		//std::cout << "Back [" << MinLaneClearBACK.at(0) << ", " << MinLaneClearBACK.at(1) << ", " << MinLaneClearBACK.at(2) << "] " << std::endl;

		if((EgoCar.GetLane() == 0) and (clearlanelengthsFWD.at(0).at(0) < clearlanelengthsFWD.at(0).at(1)) and (MinLaneClearBACK.at(1) > 10)) // clearlanelengthsBACK.at(0).at(1) < -6))
		{
			targetLane = 1;
		}
		if (EgoCar.GetLane() == 1)
		{
			if ((clearlanelengthsFWD.at(0).at(1) < clearlanelengthsFWD.at(0).at(0)) or (clearlanelengthsFWD.at(0).at(1) < clearlanelengthsFWD.at(0).at(2)))
			{
				if (clearlanelengthsFWD.at(0).at(2) < clearlanelengthsFWD.at(0).at(0))
				{
					if (MinLaneClearBACK.at(0) > 10) //(clearlanelengthsBACK.at(0).at(0) > 8)
					{
						targetLane = 0;
					}
					else if ((clearlanelengthsFWD.at(0).at(1) < clearlanelengthsFWD.at(0).at(2)) and MinLaneClearBACK.at(2) > 10) //clearlanelengthsBACK[2] < -8)
					{
						targetLane = 2;
					}
				}
				else if (MinLaneClearBACK.at(2) > 10) //(clearlanelengthsBACK[2] < -8)
				{
					targetLane = 2;
				}
			}
		}
		if ((EgoCar.GetLane() == 2) and (clearlanelengthsFWD.at(0).at(2) < clearlanelengthsFWD.at(0).at(1)) and (MinLaneClearBACK.at(1) > 10)) //clearlanelengthsBACK[1] < -8))
		{
			targetLane = 1;
		}
		//experimental to try to avoid/get out of traps on side lanes and not move over earlier
		if ((EgoCar.GetLane() == 0) and (MinLaneClearFWD.at(1) > 20) and (MinLaneClearBACK.at(1) > 10)
			and (MinLaneClearFWD.at(0) < 50) and (MinLaneClearFWD.at(2) > 60)) targetLane = 1;
		if ((EgoCar.GetLane() == 2) and (MinLaneClearFWD.at(1) > 20) and (MinLaneClearBACK.at(1) > 10)
			and (MinLaneClearFWD.at(2) < 50) and (MinLaneClearFWD.at(0) > 60)) targetLane = 1;
	}
	else
	{
		//move to middle lane whenever possible to give more options - hopefully won't cause too much weaving
		//if ((EgoCar.GetLane() != 1) and (clearlanelengthsFWD.at(0).at(1) > 50) and (MinLaneClearBACK.at(1) > 10)) targetLane = 1;
		if ((EgoCar.GetLane() != 1) and (MinLaneClearFWD.at(1) > 50) and (MinLaneClearBACK.at(1) > 10)) targetLane = 1;
		
	}
	if (targetLane > -1) PlanTargetLane = targetLane;
	return (targetLane);
}


int RoadMap::CheckForSlowCarsAhead()
{
	// if we have at least 30 m in the next X second window, everything is fine (?)
	//std::cout << "Showing " << MinLaneClearFWD.at(EgoCar.GetLane()) << " m clear ahead in lane " << EgoCar.GetLane()<< std::endl;
	if(MinLaneClearFWD.at(EgoCar.GetLane()) > 30) return -1;
	int OtherCarID = RoadMapDeck.at(OtherCarsLayer).at(0).at(EgoCar.GetLane()).at(
		    RevLimit + clearlanelengthsFWD.at(0).at(EgoCar.GetLane()));
	_loggerRoadMap->info("RoadMap: Found OtherCarID = {:d} at distance of {:d} m.", OtherCarID-1, clearlanelengthsFWD.at(0).at(EgoCar.GetLane()) );
	return OtherCarID-1;
}

int RoadMap::CheckForSlowCarsAhead(double distance)
{
	// if we have at least 30 m in the next X second window, everything is fine (?)
	//std::cout << "Showing " << MinLaneClearFWD.at(EgoCar.GetLane()) << " m clear ahead in lane " << EgoCar.GetLane() << std::endl;
	if (MinLaneClearFWD.at(EgoCar.GetLane()) > distance) return -1; //within time frame (2 secs, the car will be within buffer distance )
	int OtherCarID = RoadMapDeck.at(OtherCarsLayer).at(0).at(EgoCar.GetLane()).at(
		RevLimit + clearlanelengthsFWD.at(0).at(EgoCar.GetLane()));
	_loggerRoadMap->info("RoadMap: Found OtherCarID {:d} at distance of {:d} m.",
		OtherCarID - 1, clearlanelengthsFWD.at(0).at(EgoCar.GetLane()) );
	std::cout << "Found OtherCarID = " << OtherCarID - 1 << " at distance of " << clearlanelengthsFWD.at(0).at(EgoCar.GetLane())  << std::endl;
	return OtherCarID - 1;
}



//Try to pick best Target lane destination to try to get to in far distance maybe for A* target.
//Initial thought is that it should generally be the middle lane,
//but move to another lane if significantly clearer looking back from destination
// Test at 1 second or t_steps = 10
//TODO: maybe we should test for all timesteps?
int RoadMap::SetTarget() {

	int t_steps = 10;
	int TmpTargetLane = 1;
	std::vector<int> clearlanelengthsBACK(NumLanes);
	std::vector< std::vector <int>> RoadMapNow = RoadMapDeck[OtherCarsLayer][t_steps];


	int GoodLane = -1;
	int Horizon = 0;
	int MaxClearDistance = 15;
	for (int StepsBack = 0; StepsBack < 3 and GoodLane<0; StepsBack++) {
		for (int Lane = 0; Lane < NumLanes; Lane++) {

			auto itBACK = std::find_if(RoadMapNow[Lane].rbegin() + StepsBack * 15, RoadMapNow[Lane].rend(), [](int x) { return x > 0; });
			clearlanelengthsBACK[Lane] = std::distance(itBACK, RoadMapNow[Lane].rbegin() + StepsBack * 15);
			if (-clearlanelengthsBACK.at(Lane) > MaxClearDistance) {
				MaxClearDistance = -clearlanelengthsBACK.at(Lane);
				GoodLane = Lane;
				Horizon = FwdLimit - (StepsBack * 15);
			}
			std::cout << "At time = " << double(t_steps) * TimeIncrement << " Secs: clear back "
				<< -clearlanelengthsBACK[Lane] << " m from horizon in lane " << Lane << ".  with "
				<< StepsBack * 15 << " steps back." << std::endl;
		}
	}

	GoalLane = GoodLane;
	GoalHorizon = Horizon;
	std::cout << "chose Target of Lane " << GoodLane << " at Horizon of " << Horizon << "m." << std::endl;

	return(GoodLane);
}
