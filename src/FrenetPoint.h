//
// Created by Stanislav Olekhnovich on 13/10/2017.
// see https://github.com/fspirit/path-planning-starter
// Customized by Anthony Knight on 17/01/2018
//

#ifndef PATH_PLANNING_FRENETPOINT_H
#define PATH_PLANNING_FRENETPOINT_H

// lanes are 4m wide
const double LaneWidthInD = 4.0;
const double LaneCentreLocator = (LaneWidthInD / 2.0) * 0.95; //offset to check if car is travelling along centre of lane

															  // move centrelines of outside lanes in by 15 cm
const std::vector<double> LaneCentreD = { 2.15,6.0,9.85 };


//describe kinematics of car at specific point in time using S,D framework

struct FrenetPoint
{
    double S;
    double D;

    FrenetPoint() = default;
    FrenetPoint(double S, double D) : S(S), D(D) {}

	// return d value of centrepoint of desired lane
    inline static double LaneCenterDCoord(int laneNumber) { return LaneCentreD.at(laneNumber); };
	inline int GetLane() { 
		int lane;
		if (D < 4.0) lane = 0;
		if (D > 4.0) lane = 1;
		if (D > 8.0) lane = 2;
		return lane; // int(D / LaneWidthInD); some occurrences of D outside 0-12
	};
    inline bool IsInLane(int laneNumber) const { return D < LaneWidthInD * (laneNumber + 1)  &&
                D > LaneWidthInD * laneNumber; }

	inline bool WithinLane(int LaneNumber) const  //use to see if other cars are in a lane (even partially)
	{
		bool result = ((fabs(D - LaneCentreD.at(LaneNumber)) < 2.75) or (LaneNumber == 2 and D > LaneCentreD.at(LaneNumber)) or
			(LaneNumber == 0 and D < LaneCentreD.at(LaneNumber)));
		return result;
	}
	inline bool IsAtCenterofLane(int laneNumber) const {
		        return (fabs(D - LaneCenterDCoord(laneNumber)) < 0.1) ? true : false; }

	template<typename OStream>
	friend OStream& operator<<(OStream& os, const FrenetPoint &c)
	{
		return os << "frenet [" << c.S << " , "<< c.D <<"]";
	}
};

struct FrenetDescriptors
{
	FrenetPoint Displacement;
	FrenetPoint Velocity;
	FrenetPoint Acceleration;
	FrenetPoint Jerk;
	FrenetDescriptors() = default;
};



#endif //PATH_PLANNING_FRENETPOINT_H
