#define TMK_SETUP
#ifdef TMK_SETUP
#include <uWS/uWS.h>
#include <eigen3/Eigen/Core>
#else
#include <uWS/uWS.h>
#include "Eigen-3.3/Eigen/Core"
//#include <Eigen/Core>
#endif
#include <fstream>
#include <chrono>
#include <ctime>
#include <thread>
#include "json.hpp"

#include "spdlog/spdlog.h"
namespace spd = spdlog;
#include <memory>

//#define KEEPLANE_PATHPLANNER
//#define SPLINE_PATHPLANNER

#ifdef KEEPLANE_PATHPLANNER
#include "KeepLanePathPlanner.h"
#else
#ifdef SPLINE_PATHPLANNER
#include "SimpleSplineBasedPlanner.h"
#else
#include "JMTBasedPlanner.h"
#endif //SPLINE_PATHPLANNER
#endif //KEEPLANE_PATHPLANNER

#include "WebSocketMessageHandler.h"

using namespace std;

using std::string;
using std::cout;
using std::endl;
using std::cerr;

const int StartingLane = 1;



int main(int argc, char * argv[])
{
	uWS::Hub h;

	auto console = spd::stdout_color_st("console");
	auto my_logger = spd::basic_logger_st("PathPlannerLogger", "/mnt/c/Users/tknight/Source/Repos/CarND/CarND-T3-PathPlanning/Ubuntu_build/logs/PathPlanner.txt");
	console->info("Starting Path Planning System up");

	//HighwayMap map("../data/highway_map.csv");
	HighwayMap map("/mnt/c/Users/tknight/Source/Repos/CarND/CarND-T3-PathPlanning/data/highway_map.csv");


	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

#ifdef KEEPLANE_PATHPLANNER
	  KeepLanePathPlanner pathPlanner(map, StartingLane);
#else
#ifdef SPLINE_PATHPLANNER
	  SimpleSplineBasedPlanner pathPlanner(map, StartingLane);
#else
	JMTBasedPlanner pathPlanner(map, StartingLane);
#endif
#endif

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
}
