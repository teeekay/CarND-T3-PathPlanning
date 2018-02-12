// JMT.h
// jerk minimizing trajectory 
// taken from Udacity Courses
// Modified by Anthony M Knight 18/01/2018
//
#ifndef JMT_H
#define JMT_H

#include "FrenetPoint.h"
#include "CartesianPoint.h"
#include "spdlog/spdlog.h"

class JMT {
public:
	JMT(std::vector<double> start_s, std::vector<double> start_d, std::vector<double> end_s, std::vector<double> end_d, double T);
	FrenetPoint JMTDisplacementAt(double T);
private:
	std::vector<std::vector<double>> JMTcoefficients;
	double T;
	std::shared_ptr<spdlog::logger> _logger;
};

#endif // JMT_H
