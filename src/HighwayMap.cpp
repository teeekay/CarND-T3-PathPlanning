//
// Created by Stanislav Olekhnovich on 13/10/2017.
//
#include "HighwayMap.h"

#include <thread>
#include <uWS/uWS.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>


HighwayMap::HighwayMap(const std::string &highwayMapCsvPath)
{
	//map_logger = spdlog::get("HighwayMapLogger");
	//map_logger->flush_on(spdlog::level::info);
	//map_logger->info("System Started");
    
	ReadMapFromCsvFile(highwayMapCsvPath);
}

//CartesianPoint HighwayMap::Frenet2Cartesian(const FrenetPoint& frenetPoint) const
//{
//    int previousPoint = -1;
//
//    while(frenetPoint.S > mapPointsS.at(previousPoint+1) && (previousPoint < (int)(mapPointsS.size()-1) ))
//    {
//        previousPoint++;
//    }
//
//    auto wp2 = static_cast<int>((previousPoint + 1) % mapPointsX.size());
//
//    double heading = atan2((mapPointsY[wp2]-mapPointsY[previousPoint]),(mapPointsX[wp2]-mapPointsX[previousPoint]));
//    // the x,y,s along the segment
//    double seg_s = (frenetPoint.S-mapPointsS[previousPoint]);
//
//    double seg_x = mapPointsX[previousPoint]+seg_s*cos(heading);
//    double seg_y = mapPointsY[previousPoint]+seg_s*sin(heading);
//
//    double perp_heading = heading-M_PI_2; // PI/2.
//
//    double x = seg_x + frenetPoint.D * cos(perp_heading);
//    double y = seg_y + frenetPoint.D * sin(perp_heading);
//
//    return {x,y};
//}

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
//    int prev_wp;
//    prev_wp = next_wp-1;
//    if(next_wp == 0)
//    {
//        prev_wp  = mapPointsX.size()-1;
//    }
//
//    double n_x = mapPointsX[next_wp]-mapPointsX[prev_wp];
//    double n_y = mapPointsY[next_wp]-mapPointsY[prev_wp];
//    double x_x = cartesianPoint.X - mapPointsX[prev_wp];
//    double x_y = cartesianPoint.Y - mapPointsY[prev_wp];
//
//    // find the projection of x onto n
//    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
//    double proj_x = proj_norm*n_x;
//    double proj_y = proj_norm*n_y;
//
//    //double frenet_d = EuclidDistance(cartesianPoint, {proj_x,proj_y});  //oops
//	double frenet_d = EuclidDistance({ x_x,x_y }, { proj_x,proj_y });
//
//    //see if d value is positive or negative by comparing it to a center point
//
//    double center_x = 1000-mapPointsX[prev_wp];
//    double center_y = 2000-mapPointsY[prev_wp];
//    double centerToPos = EuclidDistance({center_x,center_y}, {x_x,x_y});
//    double centerToRef = EuclidDistance({center_x,center_y}, {proj_x,proj_y});
//
//    if(centerToPos <= centerToRef)
//    {
//        frenet_d *= -1;
//    }
//
//    // calculate s value
//    double frenet_s = 0;
//    for(int i = 0; i < prev_wp; i++)
//    {
//        frenet_s += EuclidDistance({mapPointsX[i], mapPointsY[i]}, {mapPointsX[i+1],mapPointsY[i+1]});
//    }
//
//    frenet_s += EuclidDistance({0,0} ,{proj_x,proj_y});
//
//    return {frenet_s,frenet_d};
//}

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
	mapPointsTheta.push_back(mapPointsTheta.at(0));

	

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

	//for (double s = 5400.0; s < 5800.0; s += 1.0)
	//	std::cout << s << ", " << SplineFrenetSToX(s) << ", " << SplineFrenetSToY(s) << ", " << SplineFrenetSToTheta(s) << ", " << SplineFrenetSToDX(s) << ", " << SplineFrenetSToDY(s) << std::endl;
	//exit(-1);

	//tests


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
//	// spline interpolation
//  double wp_x, wp_y, wp_dx, wp_dy, wp_theta;
//	wp_x = SplineFrenetSToX(FPt.S);
//	wp_y = SplineFrenetSToY(FPt.S);
//	wp_dx = SplineFrenetSToDX(FPt.S);
//	wp_dy = SplineFrenetSToDY(FPt.S);
//	wp_theta = SplineFrenetSToTheta(FPt.S);

	//dx and dy may not be reliable enough
//	CPt.X = wp_x + wp_dx * FPt.D;
//	CPt.Y = wp_y + wp_dy * FPt.D;

	//_logg->info("FrenetToCartesian using dx dy splines gives ( {:03.3f}, {:03.3f} ) = [ {:03.3f}, {:03.3f} ]", FPt.S, FPt.D, CPt.X, CPt.Y);
	//std::cout << "FrenetToCartesian using dx dy splines gives ( " << FPt.S << ", " << FPt.D 
	//	    << " )  - [ " << CPt.X << ", " << CPt.Y << " )" << std::endl;

	
	
//	CPt.X = wp_x + cos(wp_theta) * FPt.D;
//	CPt.Y = wp_y + sin(wp_theta) * FPt.D;

	//_logg->info("FrenetToCartesian using theta splines gives ( {:03.3f}, {:03.3f} ) = [ {:03.3f}, {:03.3f} ]", FPt.S, FPt.D, CPt.X, CPt.Y);
	//std::cout << "FrenetToCartesian using theta splines gives ( " << FPt.S << ", " << FPt.D
	//	<< " )  - [ " << CPt.X << ", " << CPt.Y << " )" << std::endl;

//	return CPt;
//}

//Attempt to convert Frenet Path to Cartesian while maintaining velocities in Frenet Path 
//(by compensating for Radius of Curvature 
std::vector<CartesianPoint> HighwayMap::ConvertCurveMaintainSpeed(std::vector<FrenetPoint> Path, CartesianPoint StartCPt) const
{
	spdlog::get("console")->debug("FrenetPath passed to ConvertCurveMaintainSpeed.");
	for (FrenetPoint lFPt : Path)
	{
		spdlog::get("console")->debug("{:03.3f}, {:03.3f}", lFPt.S, lFPt.D);
	}
	std::vector<CartesianPoint> OutPath;
	spdlog::get("console")->info("InputFrenetCurve to be adjusted");
	CartesianPoint lCPt = FrenetToCartesian(Path.at(0));
	lCPt.ThetaRads = StartCPt.ThetaRads;
	OutPath.push_back( lCPt );
	spdlog::get("console")->debug("Frenet Pt 000 {:03.3f},{:03.3f} is Cart Pt {:03.3f},{:03.3f}",Path.at(0).S, Path.at(0).D,OutPath.back().X, OutPath.back().Y);
	for (int index = 1; index < Path.size(); index++)
	{
		//distance is actually velocity over .02 seconds
		lCPt = FrenetToCartesian(Path.at(index));
		spdlog::get("console")->debug("Frenet Pt {:03d} {:03.3f},{:03.3f} is Cart Pt {:03.3f},{:03.3f} before adjustment ",index, Path.at(index).S, Path.at(index).D, lCPt.X, lCPt.Y);
		double arclength = 2.0;
		double deltaTheta = headingdifference(atan2(SplineFrenetSToDY(Path.at(index).S + arclength/2.0), SplineFrenetSToDX(Path.at(index).S + arclength / 2.0)),
			atan2(SplineFrenetSToDY(Path.at(index).S - arclength / 2.0), SplineFrenetSToDX(Path.at(index).S - arclength / 2.0) ));
		if (fabs(deltaTheta) > 0.001) //curvature adjustment needs to be applied.
		{
			double kurvatureR = arclength / deltaTheta;
			double adj = deltaTheta < 0.0 ? -1.0 : 1.0;
			double comp = (kurvatureR + adj * Path.at(index).D)/ kurvatureR - 1.0;
			double Sdiff = (Path.at(index).S - Path.at(index - 1).S)*comp;
			spdlog::get("console")->debug("Adjusting S by {:+1.4f} due to deltaTheta of {:1.4f} and Radius of {:3.0f} m.", Sdiff, deltaTheta, kurvatureR);
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
	spdlog::get("console")->debug("Adjusted FrenetPath from ConvertCurveMaintainSpeed.");
	for (FrenetPoint lFPt : Path)
	{
		spdlog::get("console")->debug("{:03.3f}, {:03.3f}", lFPt.S, lFPt.D);
	}
	spdlog::get("console")->debug("Adjusted CartesianPath from ConvertCurveMaintainSpeed.");
	for (CartesianPoint lCPt1 : OutPath)
	{
		spdlog::get("console")->debug("{:03.3f}, {:03.3f}, {:03.3f}", lCPt1.X, lCPt1.Y, lCPt1.ThetaRads);
	}
	return OutPath;
}

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




