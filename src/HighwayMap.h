//
// Created by Stanislav Olekhnovich on 13/10/2017.
// see https://github.com/fspirit/path-planning-starter
// Modified by Anthony M Knight 30/01/2018
//
// Used to load Highway data points and transform co-ords between Cartesian and Frenet

#ifndef PATH_PLANNING_HIGHWAYMAP_H
#define PATH_PLANNING_HIGHWAYMAP_H

#include <string>
#include <vector>
#include <cmath>

#include "CartesianPoint.h"
#include "FrenetPoint.h"
#include "spline.h"
#include "spdlog/spdlog.h"

#define MAX_S 6945.554

inline double RangeS(double S)
{
	while (S < 0) { S += MAX_S; }
	S = fmod(S, MAX_S);
	return S;
}

double headingdifference(double Theta1, double Theta2);

class HighwayMap
{
public:
    HighwayMap(const std::string& highwayMapCsvPath);
    CartesianPoint FrenetToCartesian(const FrenetPoint& frenetPoint) const;
    FrenetPoint CartesianToFrenet(const CartesianPoint& cartesianPoint) const;
    int NextWaypoint(CartesianPoint CurrentLocationCPt) const;
    int ClosestWaypoint(CartesianPoint CurrentLocationCPt) const;
	std::vector<CartesianPoint> ConvertCurveMaintainSpeed(std::vector<FrenetPoint>& Path, CartesianPoint StartCPt, bool SpeedControl=true) const;
	const double MaxS;
private:
	std::vector<double> mapPointsX;
	std::vector<double> mapPointsY;
	std::vector<double> mapPointsS;
	std::vector<double> mapPointsDX;
	std::vector<double> mapPointsDY;
	std::vector<double> mapPointsTheta;

	std::vector<double> DiscreteX;
	std::vector<double> DiscreteY;
	std::vector<double> DiscreteS;
	std::vector<double> DiscreteTheta;

	void ReadMapFromCsvFile(const std::string& highwayMapCsvPath);
	void LoadSSplines();

	tk::spline SplineFrenetSToX;
	tk::spline SplineFrenetSToY;
	tk::spline SplineFrenetSToDX;
	tk::spline SplineFrenetSToDY;
	tk::spline SplineFrenetSToTheta;
	std::shared_ptr<spdlog::logger> _loggerHighwayMap;
    inline double EuclidDistance(CartesianPoint p1, CartesianPoint p2) const
    {
        return sqrt((p2.X-p1.X)*(p2.X-p1.X)+(p2.Y-p1.Y)*(p2.Y-p1.Y));
    }	
};

#endif //PATH_PLANNING_HIGHWAYMAP_H
