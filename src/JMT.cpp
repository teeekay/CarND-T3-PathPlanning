// JMT.cpp code from Udacity Lessons
// Modified by Anthony M Knight 30/01/2018

#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "JMT.h"


JMT::JMT(vector< double> start_s, vector< double> start_d, vector <double> end_s, vector <double> end_d, double T)
{
	/*
	Calculate the Jerk Minimizing Trajectory that connects the initial state
	to the final state in time T.

	INPUTS

	start - the vehicles start location given as a length three array
	corresponding to initial values of [s, s_dot, s_double_dot]

	end   - the desired end state for vehicle. Like "start" this is a
	length three array.

	T     - The duration, in seconds, over which this maneuver should occur.

	OUTPUT
	an array of length 6, each value corresponding to a coefficent in the polynomial
	s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

	EXAMPLE

	> JMT( [0, 10, 0], [10, 10, 0], 1)
	[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
	*/
	_JMTL = spdlog::get("JMT");
	this->T = T;

	std::vector<std::vector<double>> StartState = { start_s, start_d };
	std::vector<std::vector<double>> EndState = { end_s, end_d };

	for (unsigned i = 0; i < StartState.size(); i++) {
		double a_0 = StartState.at(i).at(0);
		double a_1 = StartState.at(i).at(1);
		double a_2 = 0.5*StartState.at(i).at(2);
		MatrixXd a = MatrixXd(3, 3);
		MatrixXd c = MatrixXd(3, 1);

		a << T * T*T, T*T*T*T, T*T*T*T*T,
			3 * T*T, 4 * T*T*T, 5 * T*T*T*T,
			6 * T, 12 * T*T, 20 * T*T*T;

		c << EndState.at(i).at(0) - (StartState.at(i).at(0) + StartState.at(i).at(1)*T + 0.5*StartState.at(i).at(2)*T*T),
			EndState.at(i).at(1) - (StartState.at(i).at(1) + StartState.at(i).at(2)*T),
			EndState.at(i).at(2) - StartState.at(i).at(2);

		MatrixXd ai = a.inverse();
		MatrixXd b = ai * c;

		double a_3 = b(0, 0);
		double a_4 = b(1, 0);
		double a_5 = b(2, 0);

		std::vector<double> Coeffs = { a_0, a_1, a_2, a_3, a_4, a_5 };

		JMTcoefficients.push_back( Coeffs );
	}

}

//Returns location at time point t in Frenet coordinates
// 0 < t < T
FrenetPoint JMT::JMTDisplacementAt(double t) {
	//std::cout << "t is now " << t << "and T is " << T << std::endl;
	assert(0.0 <= t <= T);

	std::vector<double> Displacement;
	std::vector<double> Velocity;
	std::vector<double> Acceleration;
	std::vector<double> Jerk;
	for (std::vector<double> Coeffs : JMTcoefficients)
	{
		Displacement.push_back(Coeffs.at(0) + Coeffs.at(1)*t + Coeffs.at(2)*t*t +
			Coeffs.at(3)*t*t*t + Coeffs.at(4)*t*t*t*t + Coeffs.at(5)*t*t*t*t*t);
		Velocity.push_back(Coeffs.at(1) + Coeffs.at(2)*t +
			Coeffs.at(3)*t*t + Coeffs.at(4)*t*t*t + Coeffs.at(5)*t*t*t*t);
		Acceleration.push_back(Coeffs.at(2) + Coeffs.at(3)*t + Coeffs.at(4)*t*t
			+ Coeffs.at(5)*t*t*t);
		Jerk.push_back(Coeffs.at(3) + Coeffs.at(4)*t + Coeffs.at(5)*t*t);
	}
	FrenetPoint JMTLocationAt_t(Displacement.at(0), Displacement.at(1));

	if((Velocity.at(0) > 22.2) or (-5.0 > Acceleration.at(0) > 5.0))
	{
		_JMTL->warn("Param exceeds t,S,D,Vel_S,Vel_D,Acc_S,Acc_D,Jerk_S,Jerk_D");
		_JMTL->warn("{:02.2f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}",
			t, Displacement.at(0), Displacement.at(1), Velocity.at(0), Velocity.at(1), Acceleration.at(0), Acceleration.at(1),
			Jerk.at(0), Jerk.at(1)); 
	}
	return(JMTLocationAt_t);
}
//Returns location at time point t in Frenet coordinates
// 0 < t < T
FrenetDescriptors JMT::JMTFrenetDescriptorsAt(double t)
{
	//std::cout << "t is now " << t << "and T is " << T << std::endl;
	assert(0.0 <= t <= T);
	FrenetDescriptors FDesc;

	std::vector<double> Displacement;
	std::vector<double> Velocity;
	std::vector<double> Acceleration;
	std::vector<double> Jerk;
	for (std::vector<double> Coeffs : JMTcoefficients)
	{
		Displacement.push_back(Coeffs.at(0) + Coeffs.at(1)*t + Coeffs.at(2)*t*t +
			Coeffs.at(3)*t*t*t + Coeffs.at(4)*t*t*t*t + Coeffs.at(5)*t*t*t*t*t);
		Velocity.push_back(Coeffs.at(1) + Coeffs.at(2)*t +
			Coeffs.at(3)*t*t + Coeffs.at(4)*t*t*t + Coeffs.at(5)*t*t*t*t);
		Acceleration.push_back(Coeffs.at(2) + Coeffs.at(3)*t + Coeffs.at(4)*t*t
			+ Coeffs.at(5)*t*t*t);
		Jerk.push_back(Coeffs.at(3) + Coeffs.at(4)*t + Coeffs.at(5)*t*t);
	}
	FDesc.Displacement = { Displacement.at(0), Displacement.at(1) };
	FDesc.Velocity = {Velocity.at(0),Velocity.at(1) };
	FDesc.Acceleration = { Acceleration.at(0),Acceleration.at(1) };
	FDesc.Jerk = { Jerk.at(0), Jerk.at(1) };
	
	if ((Velocity.at(0) > 22.2) or (-5.0 > Acceleration.at(0) > 5.0))
	{
		_JMTL->warn("JMT: Param exceeds t,S,D,Vel_S,Vel_D,Acc_S,Acc_D,Jerk_S,Jerk_D:" 
		    "{:02.2f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}",
			t, Displacement.at(0), Displacement.at(1), Velocity.at(0), Velocity.at(1), Acceleration.at(0), Acceleration.at(1),
			Jerk.at(0), Jerk.at(1));
	}

	return(FDesc);
}