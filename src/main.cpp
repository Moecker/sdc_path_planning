///
/// @file
///

#include <fstream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "uWS/uWS.h"

#include "KeepLanePathPlanner.h"
#include "Playground.h"
#include "SimpleSplineBasedPlanner.h"
#include "WebSocketMessageHandler.h"

using std::cerr;
using std::cout;
using std::endl;
using std::string;

const int kStartingLane = 1;

void RunPathPlanner();

int main(int argc, char* argv[])
{
    RunBehaviorPlanner();
    StateMachineTest();
    RunPathPlanner();
}

void RunPathPlanner()
{
    uWS::Hub h;
    HighwayMap map("../data/highway_map.csv");

    // Here we can decide which path planer to choose
    SimpleSplineBasedPlanner path_planner(map, kStartingLane);
    WebSocketMessageHandler handler(path_planner);

    h.onMessage([&handler](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        if (length == 0)
            return;

        string message(data, length);
        handler.HandleMessage(message, ws);
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { cout << "Connected" << endl; });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
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
