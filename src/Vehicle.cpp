///
/// @file
/// @brief Note: Taken from Udacity quizzes
///

#include "Vehicle.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include "Cost.h"

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, double s, double v, double a, string state)
{
    this->lane_ = lane;
    this->s_ = s;
    this->v_ = v;
    this->a_ = a;
    this->state_ = state;
    max_acceleration_ = -1;
}

Vehicle::~Vehicle() {}

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
    string state = this->state_;
    if (state.compare("KL") == 0)
    {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0)
    {
        if (lane_ != available_lanes_ - 1)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane_ != 0)
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
    double max_velocity_accel_limit = this->max_acceleration_ + this->v_;
    double new_position;
    double new_velocity;
    double new_accel;

    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (GetVehicleAhead(predictions, lane, vehicle_ahead))
    {
        if (GetVehicleBehind(predictions, lane, vehicle_behind))
        {
            new_velocity = vehicle_ahead.v_;  // must travel at the speed of traffic, regardless of preferred buffer
        }
        else
        {
            double max_velocity_in_front =
                (vehicle_ahead.s_ - this->s_ - this->preferred_buffer_) + vehicle_ahead.v_ - 0.5 * (this->a_);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed_);
        }
    }
    else
    {
        new_velocity = min(max_velocity_accel_limit, this->target_speed_);
    }

    new_accel = new_velocity - this->v_;  // Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s_ + new_velocity + new_accel / 2.0;
    return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::ConstantSpeedTrajectory()
{
    /*
    Generate a constant speed trajectory.
    */
    double next_pos = PositionAt(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane_, this->s_, this->v_, this->a_, this->state_),
                                  Vehicle(this->lane_, next_pos, this->v_, 0, this->state_)};
    return trajectory;
}

vector<Vehicle> Vehicle::KeepLaneTrajectory(map<int, vector<Vehicle>> predictions)
{
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(lane_, this->s_, this->v_, this->a_, state_)};
    vector<double> kinematics = GetKinematics(predictions, this->lane_);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane_, new_s, new_v, new_a, "KL"));
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
    int new_lane = this->lane_ + lane_direction_[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane_, this->s_, this->v_, this->a_, this->state_)};
    vector<double> curr_lane_new_kinematics = GetKinematics(predictions, this->lane_);

    if (GetVehicleBehind(predictions, this->lane_, vehicle_behind))
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

    trajectory.push_back(Vehicle(this->lane_, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::LaneChangeTrajectory(string state, map<int, vector<Vehicle>> predictions)
{
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane_ + lane_direction_[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s_ == this->s_ && next_lane_vehicle.lane_ == new_lane)
        {
            // If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane_, this->s_, this->v_, this->a_, this->state_));
    vector<double> kinematics = GetKinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::IncrementFrenetForTimestep(int dt = 1)
{
    this->s_ = PositionAt(dt);
}

double Vehicle::PositionAt(int t)
{
    return this->s_ + this->v_ * t + this->a_ * t * t / 2.0;
}

bool Vehicle::GetVehicleBehind(map<int, vector<Vehicle>> predictions, int lane, Vehicle& vehicle_out)
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
        if (temp_vehicle.lane_ == this->lane_ && temp_vehicle.s_ < this->s_ && temp_vehicle.s_ > max_s)
        {
            max_s = temp_vehicle.s_;
            vehicle_out = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::GetVehicleAhead(map<int, vector<Vehicle>> predictions, int lane, Vehicle& vehicle_out)
{
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = this->target_s_;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane_ == this->lane_ && temp_vehicle.s_ > this->s_ && temp_vehicle.s_ < min_s)
        {
            min_s = temp_vehicle.s_;
            vehicle_out = temp_vehicle;
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
            next_v = PositionAt(i + 1) - s_;
        }
        predictions.push_back(Vehicle(this->lane_, next_s, next_v, 0));
    }
    return predictions;
}

void Vehicle::RealizeNextState(vector<Vehicle> trajectory)
{
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state_ = next_state.state_;
    this->lane_ = next_state.lane_;
    this->s_ = next_state.s_;
    this->v_ = next_state.v_;
    this->a_ = next_state.a_;
}

void Vehicle::Configure(vector<double> road_data)
{
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed_ = road_data[0];
    available_lanes_ = static_cast<int>(road_data[1]);
    target_s_ = road_data[2];
    target_lane_ = static_cast<int>(road_data[3]);
    max_acceleration_ = road_data[4];
}
