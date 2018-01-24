//
// Created by Stanislav Olekhnovich on 17/10/2017.
// Modified by Anthony M Knight 18/01/2018
//

#include <vector>
//#include <chrono>
#include <iostream>
#include <iomanip> //for printing roadmap.
#include <algorithm>
#include <iterator>

#include "SimpleSplineBasedPlanner.h"

#define G_FORCE_MPS 9.8
// maximum safe acceleration per step

const double DefaultAcceleration = 0.2;// .224;
const double MaxAccelerationMpSsq = 0.225 * G_FORCE_MPS; /* 0.25 times force of gravity */ //0.25 still exceeds max decelleration when approaching slow cars
const double MaxSpeedMpH = 49.0;//47.5
const double MaxSpeedinLaneChangeMpH = 48.0;

// if we plan 50 points ahead at maximum speed of 49.5 mph = 22.1 mps, and at 0.02 sec intervals, we will only go out 22.1 m.


static const int XAxisPlanningHorizon = 30;
const double CriticalThresholdInMeters = 15;
const double SimulatorRunloopPeriod = 0.02;
static const int MaxNumberOfPointsInPath = 50;
const int LeftmostLaneNumber = 0;

double deg2rad(double x) { return x * M_PI / 180; }
double MphToMetersPerSecond(double mphValue) { return mphValue * (1609.34 / 3600.0); }
double safeAcceleration(double accel) { return (fabs(accel) > MaxAccelerationMpSsq) ? fabs(accel) / accel * MaxAccelerationMpSsq : accel; }

const double MaxSpeedMpS = MphToMetersPerSecond(MaxSpeedMpH);
const double MaxSpeedInLaneChangeMpS = MphToMetersPerSecond(MaxSpeedinLaneChangeMpH);
double acceleration = 0.0;
double laststep_targetspeed = 0.0;

double currentSpeedMpS;

//STATES
//Initial
//Starting up.
//Stay in lane
//Match Speed of Car ahead
//Get ready for lane change
//Change lane



std::vector<CartesianPoint> SimpleSplineBasedPlanner::GeneratePath(PathPlannerInput input)
{
	//get a sorted list of cars in this alne in front of car that will be within 30 m.
	std::vector<OtherCar> CloseCarsinLane = IsTooCloseToOtherCar(input);

	//how fast are we going now?
	currentSpeedMpS = input.SpeedMpS;

	std::cout << "location:S,d" <<input.LocationFrenet.S <<", "<< input.LocationFrenet.D <<" : Currently going " << currentSpeedMpS << " mps or "
		<< input.SpeedMpH << " MpH. end of currentspline is at " << laststep_targetspeed << " mps." << std::endl;

	if (input.Path.size() > 5) 
	{
		std::cout << "Debug PATH - [i,S,D] - ";
		for (int i = 0; i < 5; i++)	std::cout << " [" << i << " , " << map.CartesianToFrenet(input.Path.at(i)).S << " , " << map.CartesianToFrenet(input.Path.at(i)).D << "]";
		std::cout << std::endl;
	}
	
	bool lanechange = CreateCarPlan(input.OtherCars, input.LocationFrenet);
	if (lanechange) {
		if (input.PreviousPathX.size() > 6)
		{
			input.PreviousPathX.resize(6);
			input.PreviousPathY.resize(6);
			input.Path.resize(6);
			input.PathEndpointFrenet = map.CartesianToFrenet(input.Path.back());  //this might need to be redone
		}
		double tmp_targetSpeed = currentSpeedMpS;
		if (tmp_targetSpeed > MaxSpeedInLaneChangeMpS) {
			tmp_targetSpeed = MaxSpeedInLaneChangeMpS; //reduce speed slightly in lane change)
		}
		//sometimes this blows up
		laststep_targetspeed = 50.0 * sqrt(pow((input.Path.at(input.Path.size() - 1).X - input.Path.at(input.Path.size() - 2).X),2.0) +
			pow((input.Path.at(input.Path.size() - 1).Y - input.Path.at(input.Path.size() - 2).Y), 2.0));
		std::cout << "laststep_targetspeed calculated as " << laststep_targetspeed << " with Path.size of " << input.Path.size() 
//			<< " and S1=" << map.CartesianToFrenet(input.Path.at(input.Path.size() - 5)).S << " and S0=" << map.CartesianToFrenet(input.Path.at(input.Path.size() - 6)).S 
			<< std::endl;
		acceleration = safeAcceleration(tmp_targetSpeed - laststep_targetspeed);
		std::cout << "Initiating Lane Change from Lane " << input.LocationFrenet.GetLane() << " to Lane " << targetLane << ", with acceleration of " << acceleration << std::endl;

	}
	else
	{
		if (input.PreviousPathX.size() > 40) {
			return input.Path;
		}

		if (!CloseCarsinLane.empty())
		{
			double tmp_targetSpeed = 0.99 * CloseCarsinLane[0].Speed2DMagnitudeMpS();// what about check for car going too fast or wrong direction?

			// maybe we should rebuild spline at this point to start slowing down sooner.
			if (input.PreviousPathX.size() > 10) {
				input.PreviousPathX.resize(10);
				input.PreviousPathY.resize(10);
				input.Path.resize(10);
				input.PathEndpointFrenet = map.CartesianToFrenet(input.Path.back());
			}


			//acceleration = safeAcceleration(input.SpeedMpS - tmp_targetSpeed); //change in speed over 1 second
			acceleration = safeAcceleration(laststep_targetspeed - tmp_targetSpeed); //change in speed over 1 second

			targetSpeed = laststep_targetspeed - acceleration * (MaxNumberOfPointsInPath - input.PreviousPathX.size()) / MaxNumberOfPointsInPath;

			std::cout << "Car Ahead!: Trying to slow down from " << input.SpeedMpS << " to " << laststep_targetspeed << " (at end of existing spline) to " << targetSpeed << " M/sec" << std::endl;
		}
		else if (input.SpeedMpS < MaxSpeedMpS)
		{

			acceleration = MaxAccelerationMpSsq;
			targetSpeed = (MaxSpeedMpS > laststep_targetspeed + MaxAccelerationMpSsq * (MaxNumberOfPointsInPath - input.PreviousPathX.size()) / MaxNumberOfPointsInPath)
				? laststep_targetspeed + MaxAccelerationMpSsq * (MaxNumberOfPointsInPath - input.PreviousPathX.size()) / MaxNumberOfPointsInPath : MaxSpeedMpS; // DefaultAcceleration;
			std::cout << "Increasing velocity at end of spline to target of " << targetSpeed << "m/s.  " << (MaxNumberOfPointsInPath - input.PreviousPathX.size()) << " points in planner left." << std::endl;
			//targetSpeed += getAcceleration(3);
		}
	}

	auto anchorsGenerationResult = GenerateAnchorPoints(input);
	auto anchorsLocal = ConvertPointsToLocalSystem(anchorsGenerationResult.AnchorPoints, anchorsGenerationResult.ReferencePoint);

	auto anchorsBasedSpline = GetSplineFromAnchorPoints(anchorsLocal);
	auto newPathPoints = GenerateNewPointsWithSpline(anchorsBasedSpline, (int)input.Path.size());

	laststep_targetspeed = targetSpeed;

	std::vector<CartesianPoint> outputPath = { input.Path };
	for (auto& p : newPathPoints)
		outputPath.push_back(p.ToGlobal(anchorsGenerationResult.ReferencePoint));

	return outputPath;
}


// lets change this to return car object from which we can get distance, 
// this could be used to assist with adjusting deceleration (more/less)
// make sure we return closest car in front of us in lane
//bool SimpleSplineBasedPlanner::IsTooCloseToOtherCar(const PathPlannerInput &input) const
 std::vector<OtherCar> SimpleSplineBasedPlanner::IsTooCloseToOtherCar(const PathPlannerInput &input) const
{
	 double egoPredictedEndpointS = !input.Path.empty() 
		 ? input.PathEndpointFrenet.S + (MaxNumberOfPointsInPath - input.Path.size())/ MaxNumberOfPointsInPath * input.SpeedMpS
		 : input.LocationFrenet.S + input.SpeedMpS;
	//double egoLookFwdWindow = !input.Path.empty() ? input.Path.size() * SimulatorRunloopPeriod : 30.0 * SimulatorRunloopPeriod;
	 double egoLookFwdWindow = 1.0;
	double distance = CriticalThresholdInMeters;

	std::vector<OtherCar> CloseCarsInLane;

    for (auto& otherCar : input.OtherCars)
    {
        if (otherCar.IsInLane(targetLane))
        {
            double otherCarPredictedS = otherCar.LocationFrenet.S +
				(egoLookFwdWindow * otherCar.Speed2DMagnitudeMpS());
                   //                     (input.Path.size() * SimulatorRunloopPeriod * otherCar.Speed2DMagnitudeMpS());
			if (otherCarPredictedS > egoPredictedEndpointS &&
				(otherCarPredictedS - egoPredictedEndpointS) < CriticalThresholdInMeters)
			{
				double tmp_distance = otherCarPredictedS - egoPredictedEndpointS;
				if (tmp_distance < distance)
				{
					std::cout << "Adding otherCar ID [" << otherCar.id << "] with projected distance " << tmp_distance << " m to end of queue." << std::endl;
					CloseCarsInLane.push_back(otherCar);
				}
				else
				{
					std::cout << "Inserting otherCar ID [" << otherCar.id << "] with projected distance " << tmp_distance << " m to start of queue." << std::endl;
					CloseCarsInLane.insert(CloseCarsInLane.begin(), otherCar);
					
				}
			}
        }
    }
	return CloseCarsInLane;
}

 bool SimpleSplineBasedPlanner::CreateCarPlan(std::vector<OtherCar> &Cars, FrenetPoint EgoCar) {

	 int numLanes = 3;
	 std::vector< std::vector <int>>RoadMapNow(numLanes, std::vector<int>(80, 0)); // lane;

	 RoadMapNow[EgoCar.GetLane()][floor(40 / 2)] = -1;

	 for (auto& otherCar : Cars)
	 {
		 int localS = floor((otherCar.LocationFrenet.S - EgoCar.S + 40) / 2);
		 if (localS > -1 and localS < 80)
			 RoadMapNow[otherCar.LocationFrenet.GetLane()][localS] = otherCar.id + 1;// need to offset so car 0 doesn't dispppear!
	 }
#define PRINTPLAN 1
#ifdef PRINTPLAN
	 for (int i = 0; i < numLanes; i++) {
		 for (int j = 0; j < 80; j++) {
			 if (RoadMapNow[i][j] == 0) {
				 std::cout << "**";
			 }
			 else
			 {
				 std::cout << std::setfill('0') << std::setw(2) << RoadMapNow[i][j];
			 }

		 }
		 std::cout << std::endl;
	 }
#endif

	 std::vector<int> clearlanelengthsFWD(numLanes);
	 std::vector<int> clearlanelengthsBACK(numLanes);
	 for (int i = 0; i < numLanes; i++) {
		 auto itFWD = std::find_if(RoadMapNow[i].begin()+20, RoadMapNow[i].end(), [](int x) { return x > 0; });
		 auto itBACK = std::find_if( RoadMapNow[i].rbegin()+59, RoadMapNow[i].rend(), [](int x) { return x > 0; });
		 clearlanelengthsFWD[i] = 2 * std::distance(RoadMapNow[i].begin() + 20, itFWD);
		 clearlanelengthsBACK[i] = 2 * std::distance(itBACK, RoadMapNow[i].rbegin() + 59);
		 std::cout << "Clear ahead " << clearlanelengthsFWD[i] << " m and back " << -clearlanelengthsBACK[i] << " m in lane " << i << ".  " << std::endl;
	 }
	 //std::cout << std::endl;



	 bool changeLane = false;
	 if ((EgoCar.GetLane() == targetLane) and EgoCar.IsAtCenterofLane(targetLane) ){  //make sure that car has changed into lane
		 if (clearlanelengthsFWD[EgoCar.GetLane()] < 40) {
			 if ((EgoCar.GetLane() == 0) and (clearlanelengthsFWD[0] < clearlanelengthsFWD[1]) and (clearlanelengthsBACK[1] < -6)) 
			 {
				 
				 targetLane = 1;
			 }
			 if (EgoCar.GetLane() == 1)
			 {
				 if ((clearlanelengthsFWD[1] < clearlanelengthsFWD[0]) or (clearlanelengthsFWD[1] < clearlanelengthsFWD[2]))
				 {
					 if (clearlanelengthsFWD[2] < clearlanelengthsFWD[0])
					 {
						 if (clearlanelengthsBACK[0] < -8)
						 {
							 targetLane = 0;
						 }
						 else if ((clearlanelengthsFWD[1] < clearlanelengthsFWD[2]) and clearlanelengthsBACK[2] < -8)
						 {
							 targetLane = 2;
						 }
					 }
					 else if (clearlanelengthsBACK[2] < -8)
					 {
						 targetLane = 2;
					 }
				 }
			 }
			 if ((EgoCar.GetLane() == 2) and (clearlanelengthsFWD[2] < clearlanelengthsFWD[1]) and (clearlanelengthsBACK[1] < -8))
			 {
				 targetLane = 1;
			 }
		 }
		 else 
		 {
			 //move to middle lane whenever possible to give more options - hopefully won't cause too much weaving
			 if ((EgoCar.GetLane() != 1) and (clearlanelengthsFWD[1] > 50) and (clearlanelengthsBACK[1] < -8)) targetLane = 1;
		 }
		 if (EgoCar.GetLane() != targetLane)
		 {
			 changeLane = true;
		 }
	 }
	 return (changeLane);
 }

SimpleSplineBasedPlanner::AnchorPointsGenerationResult SimpleSplineBasedPlanner::GenerateAnchorPoints(const PathPlannerInput& input) const
{
    CartesianPoint referencePoint = input.LocationCartesian;
    // FIXME: Why do we do this?
    referencePoint.Theta = deg2rad(referencePoint.Theta);



    std::vector<CartesianPoint> anchors;
    if (input.Path.empty() || input.Path.size() == 1) // if there are not at least 2 elements in the previous path to use, then we generate 2.
    {
		double delta_t = 0.02; // small time period to look back for previous location of car
        anchors.push_back({input.LocationCartesian.X - delta_t * cos(input.LocationCartesian.Theta),
                           input.LocationCartesian.Y - delta_t * sin(input.LocationCartesian.Theta)});
        anchors.push_back(referencePoint);
    }
    else
    {
		referencePoint = input.Path[input.Path.size() - 1]; // just to make more obvious .back();
        auto prevPoint = input.Path[input.Path.size() - 2];
        
        referencePoint.Theta = atan2(referencePoint.Y - prevPoint.Y, referencePoint.X - prevPoint.X);

        anchors.push_back(prevPoint);
        anchors.push_back(referencePoint);
    }

    //for (auto& i: {30, 60, 90})
	for (auto& i: {40, 60, 90})
    {
		// the distance to 
        //anchors.push_back(map.FrenetToCartesian({input.LocationFrenet.S + i, FrenetPoint::LaneCenterDCoord(targetLane)}));
		anchors.push_back(map.FrenetToCartesian({ input.LocationFrenet.S + i, FrenetPoint::LaneCenterDCoord(targetLane) }));
    }

    return { referencePoint, anchors };
}

std::vector<CartesianPoint> SimpleSplineBasedPlanner::ConvertPointsToLocalSystem(const std::vector<CartesianPoint> &anchorPointsGlobal,
                                                                                 const CartesianPoint &localReferencePoint) const
{
    std::vector<CartesianPoint> anchorPointsLocal;
    for (auto& p: anchorPointsGlobal)
    {
        anchorPointsLocal.push_back(p.ToLocal(localReferencePoint));
    }
    return anchorPointsLocal;
}

std::vector<CartesianPoint>
SimpleSplineBasedPlanner::GenerateNewPointsWithSpline(const tk::spline &newPathSpline, int pointsLeftInCurrentPath) const
{
    //const double pathEndpointX = 30;
	const double pathEndpointX = XAxisPlanningHorizon;
    double pathEndpointY = newPathSpline(pathEndpointX);
    double pathLength = sqrt(pathEndpointX * pathEndpointX + pathEndpointY * pathEndpointY);

    std::vector<CartesianPoint> pathPoints;

    double prevX = 0;
	double prevY = 0;
	double xScalingFactor = XAxisPlanningHorizon / pathLength;
	std::cout << "Scaling Factor is: " << xScalingFactor << std::endl;
    //double numberOfPoints = pathLength / (SimulatorRunloopPeriod * MphToMetersPerSecond(targetSpeed));
	//double numberOfPoints = pathLength / (SimulatorRunloopPeriod * targetSpeed);
	//double xAxisStep = XAxisPlanningHorizon / numberOfPoints;
	double xAxisStep = laststep_targetspeed * SimulatorRunloopPeriod * xScalingFactor; //xScalingFactor should prevent speedup on lane changes 
	double accelStep = 0.5 * acceleration * SimulatorRunloopPeriod * SimulatorRunloopPeriod * xScalingFactor; //xScalingFactor should prevent speedup on lane changes  

	double accelInc = accelStep;
    for (int i = 1; i <= MaxNumberOfPointsInPath - pointsLeftInCurrentPath; i++)
    {
		double x = prevX + xAxisStep + accelStep;
        double y = newPathSpline(x);
		std::cout << "Pathpoint " << i << ": " << x << ", " << y << std::endl;

		laststep_targetspeed = sqrt(pow(x-prevX,2.0)+pow(y-prevY,2.0))/ SimulatorRunloopPeriod;
		prevX = x;
		prevY = y;
		accelStep = accelStep + accelInc;
		
        pathPoints.emplace_back(x, y);
    }
	

    return pathPoints;
}

tk::spline SimpleSplineBasedPlanner::GetSplineFromAnchorPoints(const std::vector<CartesianPoint> &newPathAnchorPoints) const
{
    std::vector<double> newPathAnchorsX;
    std::vector<double> newPathAnchorsY;
    for (auto& p: newPathAnchorPoints)
    {
        newPathAnchorsX.push_back(p.X);
        newPathAnchorsY.push_back(p.Y);
    }
    tk::spline spline;
    spline.set_points(newPathAnchorsX, newPathAnchorsY);
    return spline;
}