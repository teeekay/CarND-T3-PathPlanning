//
// main.cpp
// Created by Stanislav Olekhnovich on 13/10/2017.
// see https://github.com/fspirit/path-planning-starter
// Modified by Anthony M Knight 30/01/2018
//

//#define TMK_SETUP
#include <uWS/uWS.h>
#ifdef TMK_SETUP
#include <eigen3/Eigen/Core>
#else
#include "Eigen-3.3/Eigen/Core"
#endif
#include <fstream>
#include <chrono>
#include <ctime>
#include <thread>
#include <iostream>
#include "json.hpp"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/sink.h"
#include "spdlog/fmt/ostr.h"
//namespace spd = spdlog;
#ifdef MSVS_DEBUG
#define LOGFILE "/mnt/c/Users/tknight/Source/Repos/CarND/CarND-T3-PathPlanning/Ubuntu_build/logs/PPlan.log"
#define MAPFILE "/mnt/c/Users/tknight/Source/Repos/CarND/CarND-T3-PathPlanning/data/highway_map.csv"
#else
#define LOGFILE "PPlan.log"
#define MAPFILE "../data/highway_map.csv"
#endif

#include <memory>

//#define KEEPLANE_PATHPLANNER
//#define SPLINE_PATHPLANNER

#ifdef KEEPLANE_PATHPLANNER
#include "KeepLanePathPlanner.h"
#define PATHPLANNER KeepLanePathPlanner
#else
#ifdef SPLINE_PATHPLANNER
#include "SimpleSplineBasedPlanner.h"
#define PATHPLANNER SimpleSplineBasedPlanner
#else
#include "JMTBasedPlanner.h"
#define PATHPLANNER JMTBasedPlanner
#endif //SPLINE_PATHPLANNER
#endif //KEEPLANE_PATHPLANNER

#include "WebSocketMessageHandler.h"

const int StartingLane = 1;

void SetupLogging( )
{
	try
	{
		//	auto my_logger = spd::basic_logger_st("PathPlannerLogger", );
		auto SharedFileSink = std::make_shared<spdlog::sinks::simple_file_sink_mt>("PPlan.log");
		//auto SharedFileSink = std::make_shared<spdlog::sinks::daily_file_sink_mt>("PPlan", 23, 59);
		
		auto planL = std::make_shared<spdlog::logger>("Plan", SharedFileSink);
		spdlog::register_logger(planL);
		auto jplanL = std::make_shared<spdlog::logger>("JPlan", SharedFileSink);
		spdlog::register_logger(jplanL);
		auto mainL = std::make_shared<spdlog::logger>("Main", SharedFileSink);
		spdlog::register_logger(mainL);
		auto trajL = std::make_shared<spdlog::logger>("Traj", SharedFileSink);
		spdlog::register_logger(trajL);
		auto mapL = std::make_shared<spdlog::logger>("Map", SharedFileSink);
		spdlog::register_logger(mapL);
		auto predL = std::make_shared<spdlog::logger>("Pred", SharedFileSink);
		spdlog::register_logger(predL);
		auto pathtrackL = std::make_shared<spdlog::logger>("PTL", SharedFileSink);
		spdlog::register_logger(pathtrackL);
		auto jmtL = std::make_shared<spdlog::logger>("JMT", SharedFileSink);
		spdlog::register_logger(jmtL);
	}
	catch (const spdlog::spdlog_ex& ex)
	{
		std::cout << "Log initialization failed: " << ex.what( ) << std::endl;
	}
}






int main(int argc, char * argv[])
{
	uWS::Hub h;

	auto console = spdlog::stdout_color_st("console");
	console->info("Starting Path Planning System up");
	SetupLogging( );
	console->info("Logging set up.");

	auto ML = spdlog::get("Main");
	ML->info("System starting Up.");
	
	HighwayMap map(MAPFILE);

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	PATHPLANNER pathPlanner(map, StartingLane);

	WebSocketMessageHandler handler(pathPlanner);

	h.onMessage([&handler,&start,&console](uWS::WebSocket<uWS::SERVER> ws,
						   char * data,
						   size_t length,
						   uWS::OpCode opCode)
				{
		            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
					if (length <= 2) // sometimes get messages containing just "2"
						return;
					string message (data, length);
					//console->info("Msg Received at {} mS after startup.", std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
					handler.HandleMessage(message, ws);
				});

	h.onConnection([&h,&console](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
				   {
					   console->info("WebSocket Connected");
				   });

	h.onDisconnection([&h,&console](uWS::WebSocket<uWS::SERVER> ws, int code, char * message, size_t length)
					  {
						  ws.close();
						  console->info("WebSocket Disconnected");
					  });

	const int port = 4567;
	if (h.listen(port))
		console->info("Listening to port {}.", port);
	else
		console->error ("Failed to listen to port", port);

	h.run();
	ML->warn("Goodbye");
}
