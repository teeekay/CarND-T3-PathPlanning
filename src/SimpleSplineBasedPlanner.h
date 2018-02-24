//
// Created by Stanislav Olekhnovich on 17/10/2017.
// see https://github.com/fspirit/path-planning-starter
// Updated by Anthony M Knight on 25/01/2017
//

#ifndef PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H
#define PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H

#include "PathPlanner.h"
#include "spline.h"


const double SplineMaxSpeedMpH = 49.0;//47.5
const double SplineMaxSpeedinLaneChangeMpH = 48.0;

class SimpleSplineBasedPlanner : public PathPlanner
{
public:
    explicit SimpleSplineBasedPlanner(const HighwayMap &map, int startingLane): PathPlanner(map, startingLane), targetSpeed(.0),
		MaxSpeedMpS(MphToMetersPerSecond(SplineMaxSpeedMpH)), MaxSpeedInLaneChangeMpS (MphToMetersPerSecond(SplineMaxSpeedinLaneChangeMpH)),
		acceleration(0.0), laststep_targetspeed(0.0) {};
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
private:
    double targetSpeed;
	double MaxSpeedInLaneChangeMpS;
	double MaxSpeedMpS;
	double acceleration;
	double laststep_targetspeed;
	double currentSpeedMpS;

	double GetSafeAcceleration(double accel);

	std::vector<OtherCar> IsTooCloseToOtherCar(const PathPlannerInput &input) const;
	//bool IsTooCloseToOtherCar(const PathPlannerInput &input) const;

	bool CreateCarPlan(std::vector<OtherCar> &Cars, FrenetPoint EgoCar);

    std::vector<CartesianPoint> ConvertPointsToLocalSystem(const std::vector<CartesianPoint> &newPathAnchorPoints,
                               const CartesianPoint &localReferencePoint) const;

    tk::spline GetSplineFromAnchorPoints(const std::vector<CartesianPoint> &newPathAnchorPoints) const;

    std::vector<CartesianPoint> GenerateNewPointsWithSpline(const tk::spline &newPathSpline, int pointsLeftInCurrentPath) const;

    struct AnchorPointsGenerationResult
    {
        CartesianPoint ReferencePoint;
        std::vector<CartesianPoint> AnchorPoints;

        AnchorPointsGenerationResult(const CartesianPoint &ReferencePoint, const std::vector<CartesianPoint> &AnchorPoints)
                : ReferencePoint(ReferencePoint), AnchorPoints(AnchorPoints) {}
    };

    AnchorPointsGenerationResult GenerateAnchorPoints(const PathPlannerInput& input) const;
};



#endif //PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H
