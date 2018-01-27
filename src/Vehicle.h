#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace std;

class Vehicle
{
  public:
    Vehicle();
    Vehicle(int lane, double s, double v, double a, string state = "CS");
    virtual ~Vehicle();

    vector<Vehicle> ChooseNextState(map<int, vector<Vehicle>> predictions);
    vector<string> SuccessorStates();
    vector<Vehicle> GenerateTrajectory(string state, map<int, vector<Vehicle>> predictions);
    vector<double> GetKinematics(map<int, vector<Vehicle>> predictions, int lane);

    vector<Vehicle> ConstantSpeedTrajectory();
    vector<Vehicle> KeepLaneTrajectory(map<int, vector<Vehicle>> predictions);
    vector<Vehicle> LaneChangeTrajectory(string state, map<int, vector<Vehicle>> predictions);
    vector<Vehicle> PrepareLaneChangeTrajectory(string state, map<int, vector<Vehicle>> predictions);

    void Increment(int dt);
    double PositionAt(int t);
    bool GetVehicleBehind(map<int, vector<Vehicle>> predictions, int lane, Vehicle& rVehicle);
    bool GetVehicleAhead(map<int, vector<Vehicle>> predictions, int lane, Vehicle& rVehicle);

    vector<Vehicle> GeneratePredictions(int horizon = 2);
    void RealizeNextState(vector<Vehicle> trajectory);
    void Configure(vector<int> road_data);

    string state;
    double target_speed;

    int lane;
    double s;
    double v;

    int goal_lane;
    double goal_s;

  private:
    map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};
    int preferred_buffer = 6;  // impacts "keep lane" behavior.

    double a;

    int lanes_available;
    double max_acceleration;
};

#endif
