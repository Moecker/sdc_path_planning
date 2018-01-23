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

    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
    vector<string> successor_states();
    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
    vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

    vector<Vehicle> constant_speed_trajectory();
    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);
    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    void increment(int dt);
    double position_at(int t);
    bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle& rVehicle);
    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle& rVehicle);

    vector<Vehicle> generate_predictions(int horizon = 2);
    void realize_next_state(vector<Vehicle> trajectory);
    void configure(vector<int> road_data);

    map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

    int preferred_buffer = 6;  // impacts "keep lane" behavior.

    int lane;
    double s;
    double v;

    double a;
    double target_speed;
    int lanes_available;

    double max_acceleration;
    int goal_lane;

    double goal_s;

    string state;
};

#endif
