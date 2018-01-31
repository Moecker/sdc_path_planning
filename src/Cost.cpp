///
/// @file
///

#include "Cost.h"
#include <functional>

const double kReachGoal = pow(10, 6);
const double kEfficiency = pow(10, 5);

double GoalDistanceCost(const Vehicle& vehicle,
                        const vector<Vehicle>& trajectory,
                        const map<int, vector<Vehicle>>& predictions,
                        map<string, int>& data)
{
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
    double cost;
    double distance = data["distance_to_goal"];
    if (distance > 0)
    {
        cost = 1 - 2 * exp(-(abs(2.0 * vehicle.target_lane_ - data["intended_lane"] - data["final_lane"]) / distance));
    }
    else
    {
        cost = 1;
    }
    return cost;
}

double InefficiencyCost(const Vehicle& vehicle,
                        const vector<Vehicle>& trajectory,
                        const map<int, vector<Vehicle>>& predictions,
                        map<string, int>& data)
{
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's
    target speed.
    */

    double proposed_speed_intended = GetLaneSpeed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0)
    {
        proposed_speed_intended = vehicle.target_speed_;
    }

    double proposed_speed_final = GetLaneSpeed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0)
    {
        proposed_speed_final = vehicle.target_speed_;
    }

    double cost =
        (2.0 * vehicle.target_speed_ - proposed_speed_intended - proposed_speed_final) / vehicle.target_speed_;

    return cost;
}

double GetLaneSpeed(const map<int, vector<Vehicle>>& predictions, int lane)
{
    /*
    All non ego vehicles_ in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    */
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.lane_ == lane && key != -1)
        {
            return vehicle.v_;
        }
    }
    // Found no vehicle in the lane
    return -1.0;
}

double CalculateCost(const Vehicle& vehicle,
                     const map<int, vector<Vehicle>>& predictions,
                     const vector<Vehicle>& trajectory)
{
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, int> trajectory_data = GetTrajectoryMetaData(vehicle, trajectory, predictions);
    double cost = 0.0;

    // Add additional cost functions here.
    vector<
        function<double(const Vehicle&, const vector<Vehicle>&, const map<int, vector<Vehicle>>&, map<string, int>&)>>
        cf_list = {GoalDistanceCost, InefficiencyCost};
    vector<double> weight_list = {kReachGoal, kEfficiency};

    for (int i = 0; i < cf_list.size(); i++)
    {
        double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }

    return cost;
}

map<string, int> GetTrajectoryMetaData(const Vehicle& vehicle,
                                       const vector<Vehicle>& trajectory,
                                       const map<int, vector<Vehicle>>& predictions)
{
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.
    distance_to_goal: the distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    map<string, int> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    int intended_lane;

    if (trajectory_last.state_.compare("PLCL") == 0)
    {
        intended_lane = trajectory_last.lane_ + 1;
    }
    else if (trajectory_last.state_.compare("PLCR") == 0)
    {
        intended_lane = trajectory_last.lane_ - 1;
    }
    else
    {
        intended_lane = trajectory_last.lane_;
    }

    double distance_to_goal = vehicle.target_s_ - trajectory_last.s_;
    int final_lane = trajectory_last.lane_;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = static_cast<int>(distance_to_goal);
    return trajectory_data;
}
