#include "Vehicle.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include "Cost.h"

Vehicle::Vehicle()
{
}

Vehicle::Vehicle(int lane, double s, double v, double a, string state)
{
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;
}

Vehicle::~Vehicle()
{
}

vector<Vehicle> Vehicle::ChooseNextState(map<int, vector<Vehicle>> predictions)
{
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
    vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
    the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = SuccessorStates();
    double cost;
    vector<double> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        vector<Vehicle> trajectory = GenerateTrajectory(*it, predictions);
        if (trajectory.size() != 0)
        {
            cost = CalculateCost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = static_cast<int>(distance(begin(costs), best_cost));
    return final_trajectories[best_idx];
}

vector<string> Vehicle::SuccessorStates()
{
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (state.compare("KL") == 0)
    {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane != 0)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::GenerateTrajectory(string state, map<int, vector<Vehicle>> predictions)
{
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0)
    {
        trajectory = ConstantSpeedTrajectory();
    }
    else if (state.compare("KL") == 0)
    {
        trajectory = KeepLaneTrajectory(predictions);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        trajectory = LaneChangeTrajectory(state, predictions);
    }
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
    {
        trajectory = PrepareLaneChangeTrajectory(state, predictions);
    }
    return trajectory;
}

vector<double> Vehicle::GetKinematics(map<int, vector<Vehicle>> predictions, int lane)
{
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double new_position;
    double new_velocity;
    double new_accel;

    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (GetVehicleAhead(predictions, lane, vehicle_ahead))
    {

        if (GetVehicleBehind(predictions, lane, vehicle_behind))
        {
            new_velocity = vehicle_ahead.v;  // must travel at the speed of traffic, regardless of preferred buffer
        }
        else
        {
            double max_velocity_in_front =
                (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    }
    else
    {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->v;  // Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::ConstantSpeedTrajectory()
{
    /*
    Generate a constant speed trajectory.
    */
    double next_pos = PositionAt(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state),
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::KeepLaneTrajectory(map<int, vector<Vehicle>> predictions)
{
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<double> kinematics = GetKinematics(predictions, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::PrepareLaneChangeTrajectory(string state, map<int, vector<Vehicle>> predictions)
{
    /*
    Generate a trajectory preparing for a lane change.
    */
    double new_s;
    double new_v;
    double new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<double> curr_lane_new_kinematics = GetKinematics(predictions, this->lane);

    if (GetVehicleBehind(predictions, this->lane, vehicle_behind))
    {
        // Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    }
    else
    {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = GetKinematics(predictions, new_lane);
        // Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
        {
            best_kinematics = next_lane_new_kinematics;
        }
        else
        {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::LaneChangeTrajectory(string state, map<int, vector<Vehicle>> predictions)
{
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane)
        {
            // If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<double> kinematics = GetKinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::Increment(int dt = 1)
{
    this->s = PositionAt(dt);
}

double Vehicle::PositionAt(int t)
{
    return this->s + this->v * t + this->a * t * t / 2.0;
}

bool Vehicle::GetVehicleBehind(map<int, vector<Vehicle>> predictions, int lane, Vehicle& rVehicle)
{
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1.0;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s)
        {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::GetVehicleAhead(map<int, vector<Vehicle>> predictions, int lane, Vehicle& rVehicle)
{
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s)
        {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::GeneratePredictions(int horizon)
{
    /*
    Generates predictions for non-ego vehicles_ to be used
    in trajectory generation for the ego vehicle.
    */
    vector<Vehicle> predictions;
    for (int i = 0; i < horizon; i++)
    {
        double next_s = PositionAt(i);
        double next_v = 0;
        if (i < horizon - 1)
        {
            next_v = PositionAt(i + 1) - s;
        }
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }
    return predictions;
}

void Vehicle::RealizeNextState(vector<Vehicle> trajectory)
{
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::Configure(vector<int> road_data)
{
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}
