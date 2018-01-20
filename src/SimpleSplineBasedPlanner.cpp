//
// Created by Stanislav Olekhnovich on 17/10/2017.
// Modified by Anthony M Knight 18/01/2018
//

#include <vector>
#include <iostream>

#include "SimpleSplineBasedPlanner.h"

#define G_FORCE_MPS 9.8

double deg2rad(double x) { return x * M_PI / 180; }
double MphToMetersPerSecond(double mphValue) { return mphValue * (1609.34 / 3600.0); }

// maximum safe acceleration per step
const double DefaultAcceleration = 0.2;// .224;
//const double DefaultAccelerationMPS = 0.5 * G_FORCE_MPS; /* 0.5 times force of gravity */
const double MaxSpeed = 49.5;
//const double MaxSpeedMPH = 49.5;

static const int XAxisPlanningHorizon = 30;
const double CriticalThresholdInMeters = 30;
const double SimulatorRunloopPeriod = 0.02;
static const int MaxNumberOfPointsInPath = 50;
const int LeftmostLaneNumber = 0;

std::vector<CartesianPoint> SimpleSplineBasedPlanner::GeneratePath(PathPlannerInput input)
{
    if (IsTooCloseToOtherCar(input))
    {
        targetSpeed -= DefaultAcceleration;
		//targetSpeed -= getAcceleration(3);
        targetLane = LeftmostLaneNumber;
    }
    //else if (targetSpeed < MaxSpeedMPH)
	else if (targetSpeed < MaxSpeed)
    {
        targetSpeed += DefaultAcceleration;
		//targetSpeed += getAcceleration(3);
    }

    auto anchorsGenerationResult = GenerateAnchorPoints(input);
    auto anchorsLocal = ConvertPointsToLocalSystem(anchorsGenerationResult.AnchorPoints, anchorsGenerationResult.ReferencePoint);
    
    auto anchorsBasedSpline = GetSplineFromAnchorPoints(anchorsLocal);
    auto newPathPoints = GenerateNewPointsWithSpline(anchorsBasedSpline, (int)input.Path.size());

    std::vector<CartesianPoint> outputPath = { input.Path };
    for (auto& p: newPathPoints)
        outputPath.push_back(p.ToGlobal(anchorsGenerationResult.ReferencePoint));

    return outputPath;
}


// lets change this to return car object from which we can get distance, 
// this could be used to assist with adjusting deceleration (more/less)
bool SimpleSplineBasedPlanner::IsTooCloseToOtherCar(const PathPlannerInput &input) const
{
    double egoPredictedEndpointS = !input.Path.empty() ? input.PathEndpointFrenet.S : input.LocationFrenet.S;

    for (auto& otherCar : input.OtherCars)
    {
        if (otherCar.IsInLane(targetLane))
        {
            double otherCarPredictedS = otherCar.LocationFrenet.S +
                                        (input.Path.size() * SimulatorRunloopPeriod * otherCar.Speed2DMagnitudeMpS());
            if (otherCarPredictedS > egoPredictedEndpointS &&
                    (otherCarPredictedS - egoPredictedEndpointS) < CriticalThresholdInMeters)
                return true;
        }
    }
    return false;
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

    for (auto& i: {30, 60, 90})
    {
        anchors.push_back(map.FrenetToCartesian({input.LocationFrenet.S + i, FrenetPoint::LaneCenterDCoord(targetLane)}));
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
    const double pathEndpointX = 30;
    double pathEndpointY = newPathSpline(pathEndpointX);
    double pathLength = sqrt(pathEndpointX * pathEndpointX + pathEndpointY * pathEndpointY);

    std::vector<CartesianPoint> pathPoints;

    double prevX = 0;
    double numberOfPoints = pathLength / (SimulatorRunloopPeriod * MphToMetersPerSecond(targetSpeed));
    double xAxisStep = XAxisPlanningHorizon / numberOfPoints;
    for (int i = 1; i <= MaxNumberOfPointsInPath - pointsLeftInCurrentPath; i++)
    {
        double x = prevX + xAxisStep;
        double y = newPathSpline(x);

        prevX = x;
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



