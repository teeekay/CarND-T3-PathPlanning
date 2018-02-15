//
// Created by Stanislav Olekhnovich on 13/10/2017.
//

//#include <thread>
//#include <uWS/uWS.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include "HighwayMap.h"

HighwayMap::HighwayMap(const std::string &highwayMapCsvPath)
{
	try
	{
		_loggerHighwayMap = spdlog::get("PathPlannerLogger");
	}
	catch (const spdlog::spdlog_ex& ex)
	{
		std::cout << "Log initialization failed: " << ex.what() << std::endl;
	}
	_loggerHighwayMap->info("Loading Highway Map");
	ReadMapFromCsvFile(highwayMapCsvPath);
}


FrenetPoint HighwayMap::CartesianToFrenet(const CartesianPoint& CPt) const
{
	FrenetPoint FPt;
	//int next_wp = NextWaypoint(cartesianPoint);
	int wp_1 = ClosestWaypoint(CPt);
	int wp_2 = RangeS(wp_1 + 1);
	int wp_0 = RangeS(wp_1 - 1);

	double dist0X = EuclidDistance(CPt, { DiscreteX.at(wp_0),DiscreteY.at(wp_0) });
	double dist2X = EuclidDistance(CPt, { DiscreteX.at(wp_2),DiscreteY.at(wp_2) });
	
	if (dist2X > dist0X)
	{
		wp_2 = wp_1;
		wp_1 = wp_0;
	}

	double vx_wp = DiscreteX.at(wp_2) - DiscreteX.at(wp_1);
	double vy_wp = DiscreteY.at(wp_2) - DiscreteY.at(wp_1);

	double vx_pos = CPt.X - DiscreteX.at(wp_1);
	double vy_pos = CPt.Y - DiscreteY.at(wp_1);

	double norm_wp = sqrt((vx_wp*vx_wp + vy_wp * vy_wp));
	double scalar_proj = ((vx_pos*vx_wp + vy_pos * vy_wp) / norm_wp);

	if (scalar_proj < 0)
	{
		scalar_proj = 0.0;
		FPt.D = EuclidDistance({ DiscreteX.at(wp_1),DiscreteY.at(wp_1) }, CPt);
	}
	else if (scalar_proj > norm_wp)
	{
		scalar_proj = norm_wp;
		FPt.D = EuclidDistance({ DiscreteX.at(wp_2),DiscreteY.at(wp_2) }, CPt);
	}
	else
	{
		double dist1X = EuclidDistance({ DiscreteX.at(wp_1),DiscreteY.at(wp_1) }, CPt);
		FPt.D = sqrt((dist1X*dist1X) + (scalar_proj*scalar_proj));
	}

	FPt.S = DiscreteS.at(wp_1) + scalar_proj;
	return FPt;
}

int HighwayMap::NextWaypoint(CartesianPoint currentVehicleLocation) const
{
    int closestWaypoint = ClosestWaypoint(currentVehicleLocation);

    double map_x = DiscreteX[closestWaypoint];
    double map_y = DiscreteY[closestWaypoint];

    double heading = atan2( (map_y- currentVehicleLocation.Y),(map_x- currentVehicleLocation.X) );

    double angle = std::abs(currentVehicleLocation.ThetaRads*M_PI/180.0 - heading);
	angle = std::min(2 * M_PI - angle, angle); //added to match baumanb fix

    if(angle > M_PI_4)  // > PI/4
    {
        closestWaypoint++;
    }
	if (closestWaypoint >= mapPointsX.size())
	{
	  closestWaypoint = 0;
	}

    return closestWaypoint;
}


/*Brute force Routine to locate closest point in loop to any xy point*/
int HighwayMap::ClosestWaypoint(CartesianPoint currentVehicleLocation) const
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < DiscreteX.size(); i++)
    {
        double map_x = DiscreteX[i];
        double map_y = DiscreteY[i];
        double dist = EuclidDistance(currentVehicleLocation, {map_x, map_y});
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;
}

void HighwayMap::ReadMapFromCsvFile(const std::string &highwayMapCsvPath)
{
    std::ifstream in_map_(highwayMapCsvPath.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x, y, s, d_x, d_y;
        iss >> x >> y >> s >> d_x >> d_y;

        mapPointsX.push_back(x);
        mapPointsY.push_back(y);
        mapPointsS.push_back(s);
        mapPointsDX.push_back(d_x);
        mapPointsDY.push_back(d_y);
		mapPointsTheta.push_back(atan2(d_y, d_x));
    }
	LoadSSplines();
}

void HighwayMap::LoadSSplines() {

	//spline values of X, Y, DX, DY against S., then can retrieve good interpolation between waypoints!

	// Reload initial pt as endpoint when frenet loops round to beginning
	double endpoint = MAX_S;// 6945.554;
	mapPointsS.push_back(endpoint);
	mapPointsX.push_back(mapPointsX.at(0));
	mapPointsY.push_back(mapPointsY.at(0));
	mapPointsDX.push_back(mapPointsDX.at(0));
	mapPointsDY.push_back(mapPointsDY.at(0));
	mapPointsTheta.push_back(mapPointsTheta.at(0));  //watch for wraparound on this

	//Set up splines
	SplineFrenetSToX.set_points(mapPointsS, mapPointsX);
	SplineFrenetSToY.set_points(mapPointsS, mapPointsY);
	SplineFrenetSToDX.set_points(mapPointsS, mapPointsDX);
	SplineFrenetSToDY.set_points(mapPointsS, mapPointsDY);
	SplineFrenetSToTheta.set_points(mapPointsS, mapPointsTheta);

	for (double s = 0.0; s < endpoint; s += 1.0)
	{
		DiscreteX.push_back(SplineFrenetSToX(s));
		DiscreteY.push_back(SplineFrenetSToY(s));
		DiscreteS.push_back(s);
		DiscreteTheta.push_back(SplineFrenetSToTheta(s));

	}
}


CartesianPoint HighwayMap::FrenetToCartesian(const FrenetPoint& FPt) const
{
	CartesianPoint CPt;

	double localS = RangeS(FPt.S);

	int wp_1 = (int)localS; //waypoint before point
	int wp_2 = (int)RangeS(localS + 1.0); //waypoint beyond point
	int wp_3 = (int)RangeS(localS + 2.0); //next waypoint beyond point
	int wp_0 = (int)RangeS(localS - 1.0); //next waypoint beyond point

	double theta_12 = atan2(DiscreteY.at(wp_2) - DiscreteY.at(wp_1), DiscreteX.at(wp_2) - DiscreteX.at(wp_1));
	double theta_02 = atan2(DiscreteY.at(wp_2) - DiscreteY.at(wp_0), DiscreteX.at(wp_2) - DiscreteX.at(wp_0));
	double theta_13 = atan2(DiscreteY.at(wp_3) - DiscreteY.at(wp_1), DiscreteX.at(wp_3) - DiscreteX.at(wp_1));

	double seg_s = fmod(localS, 1.0);
	double seg_x = DiscreteX.at(wp_1) + (localS - double(wp_1))*cos(theta_12);
	double seg_y = DiscreteY.at(wp_1) + (localS - double(wp_1))*sin(theta_12);

	double cos_interp = (1 - seg_s) * cos(theta_02) + seg_s * cos(theta_13);
	double sin_interp = (1 - seg_s) * sin(theta_02) + seg_s * sin(theta_13);

	double theta_interp = atan2(sin_interp, cos_interp);

	double theta_perp = theta_interp - M_PI_2;

	CPt.X = seg_x + FPt.D * cos(theta_perp);
	CPt.Y = seg_y + FPt.D * sin(theta_perp);
	return CPt;
}


//Attempt to convert Frenet Path to Cartesian while maintaining velocities in Frenet Path 
//(by compensating for Radius of Curvature 
std::vector<CartesianPoint> HighwayMap::ConvertCurveMaintainSpeed(std::vector<FrenetPoint> &Path, CartesianPoint StartCPt) const
{
	_loggerHighwayMap->info("HighwayMap: FrenetPath passed to ConvertCurveMaintainSpeed with original Cartesian translation.");
	for (FrenetPoint & lFPt : Path)
	{
		CartesianPoint lCPt1 = FrenetToCartesian(lFPt);
		_loggerHighwayMap->info("HighwayMap: {:03.3f}, {:03.3f} -> {:03.3f}, {:03.3f}", lFPt.S, lFPt.D, lCPt1.X, lCPt1.Y);
	}
	std::vector<CartesianPoint> OutPath;
	CartesianPoint lCPt = FrenetToCartesian(Path.at(0));
	lCPt.ThetaRads = StartCPt.ThetaRads;
	OutPath.push_back( lCPt );
	_loggerHighwayMap->info("HighwayMap: Frenet Pt 000 {:03.3f},{:03.3f} is Cart Pt {:03.3f},{:03.3f}",Path.at(0).S, Path.at(0).D,OutPath.back().X, OutPath.back().Y);
	for (int index = 1; index < Path.size(); index++)
	{
		//distance is actually velocity over .02 seconds
		lCPt = FrenetToCartesian(Path.at(index));
		_loggerHighwayMap->info("HighwayMap: Frenet Pt {:03d} {:03.3f}, {:03.3f} is Cart Pt {:03.3f}, {:03.3f} before adjustment.", 
			    index, Path.at(index).S, Path.at(index).D, lCPt.X, lCPt.Y);
		double arclength = 2.0; //try averaging over a longer distance to eliminate bumps.
		double deltaTheta = headingdifference(atan2(SplineFrenetSToDY(Path.at(index).S + arclength/2.0), SplineFrenetSToDX(Path.at(index).S + arclength/2.0)),
			atan2(SplineFrenetSToDY(Path.at(index).S - arclength / 2.0), SplineFrenetSToDX(Path.at(index).S - arclength / 2.0) ));
		if (fabs(deltaTheta) > 0.001) //curvature adjustment needs to be applied.
		{
			double kurvatureR = arclength / deltaTheta;
			double adj = deltaTheta < 0.0 ? -1.0 : 1.0;
			double comp = (kurvatureR + adj * Path.at(index).D)/ kurvatureR - 1.0;
			double Sdiff = (Path.at(index).S - Path.at(index - 1).S)*comp;
			_loggerHighwayMap->info("HighwayMap: Adjusting S by {:+1.4f} due to deltaTheta of {:1.4f} and Radius of {:+3.0f} m.", Sdiff, deltaTheta, kurvatureR);
			for(int j=index; j<Path.size(); j++)
			{
				Path.at(j).S -= Sdiff;
			}

		}
		lCPt = FrenetToCartesian(Path.at(index));
		lCPt.ThetaRads = atan2(lCPt.Y - OutPath.back().Y, lCPt.X - OutPath.back().X);
		OutPath.push_back(lCPt);
		spdlog::get("console")->debug("Frenet Pt {:03d} {:03.3f},{:03.3f} is Cart Pt {:03.3f},{:03.3f}", index, Path.at(index).S, Path.at(index).D, OutPath.back().X, OutPath.back().Y);
	}
	_loggerHighwayMap->info("HighwayMap: Adjusted FrenetPath after ConvertCurveMaintainSpeed.");
	for (FrenetPoint & lFPt : Path)
	{
		CartesianPoint lCPt1 = FrenetToCartesian(lFPt);
		_loggerHighwayMap->info("HighwayMap: {:03.3f}, {:03.3f} -> {:03.3f}, {:03.3f}", lFPt.S, lFPt.D, lCPt1.X, lCPt1.Y);
	}

	return OutPath;
}


// return the difference between two headings taking into account 
// the wraparound from -PI to PI
double headingdifference(double Theta1, double Theta2)
{
	double result;

	//correct for out of -PI Pi range
	Theta1 = Theta1 + M_PI;
	Theta2 = Theta2 + M_PI;
	while (Theta1 < 0) Theta1 = Theta1 + 2 * M_PI;
	while (Theta1 >= 2 * M_PI) Theta1 = Theta1 - 2 * M_PI;
	while (Theta2 < 0) Theta2 = Theta2 + 2 * M_PI;
	while (Theta2 >= 2 * M_PI) Theta2 = Theta2 - 2 * M_PI;

	//both should be in range 0 to 2PI now.
	double diff = Theta2 - Theta1;
	double absdiff = abs(diff);

	if (absdiff <= M_PI)
	{
		result = absdiff < M_PI ? diff : absdiff;
	}
	else
	{
		result = diff < 0 ? diff + 2 * M_PI : diff - 2 * M_PI;
	}
	return result;
}




