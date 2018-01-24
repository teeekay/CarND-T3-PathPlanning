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

//#define KEEPLANE_PATHPLANNER;
//#define SPLINE_PATHPLANNER 2

#ifdef KEEPLANE_PATHPLANNER
#include "KeepLanePathPlanner.h"
#else
#include "SimpleSplineBasedPlanner.h"
#endif
#include "WebSocketMessageHandler.h"

using std::string;
using std::cout;
using std::endl;
using std::cerr;

const int StartingLane = 1;



int main(int argc, char * argv[])
{
	uWS::Hub h;
	HighwayMap map("../data/highway_map.csv");
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

#ifdef KEEPLANE_PATHPLANNER
	  KeepLanePathPlanner pathPlanner(map, StartingLane);
#else
	  SimpleSplineBasedPlanner pathPlanner(map, StartingLane);
#endif

	WebSocketMessageHandler handler(pathPlanner);

	h.onMessage([&handler,&start](uWS::WebSocket<uWS::SERVER> ws,
						   char * data,
						   size_t length,
						   uWS::OpCode opCode)
				{
		            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
					if (length <= 2) // sometimes get messages containing just "2"
						return;

					string message (data, length);
					std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()
						<< " mSecs since initiation - msg recived." << std::endl;
						//"\ninbound message received ->\n" << message << "\n <- end of message." << endl;
					handler.HandleMessage(message, ws);
				});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
				   {
					   cout << "Connected" << endl;
				   });

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char * message, size_t length)
					  {
						  ws.close();
						  cout << "Disconnected" << endl;
					  });

	const int port = 4567;
	if (h.listen(port))
		cout << "Listening to port " << port << endl;
	else
		cerr << "Failed to listen to port" << endl;

	h.run();
}
