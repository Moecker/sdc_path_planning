///
/// @file
///

#include <FsmList.h>
#include "Road.h"

void RunBehaviorPlanner()
{
    using namespace std;

    // impacts default behavior for most states
    int kSpeedLimit = 10;

    // all traffic in lane (besides ego) follow these speeds
    vector<double> kLaneSpeeds = {6, 7, 8, 9};

    // Number of available "cells" which should have traffic
    double kTrafficDensity = 0.15;

    // At each timestep, ego can set acceleration to value between
    int kMaximumAcceleration = 2;

    // s value and lane number of goal.
    vector<int> kGoal = {300, 0};

    // These affect the visualization
    int kAmountOfRoadVisible = 40;

    Road road = Road(kLaneSpeeds);

    road.update_width_ = kAmountOfRoadVisible;

    road.PopulateTraffic();

    int goal_s = kGoal[0];
    int goal_lane = kGoal[1];

    int num_lanes = static_cast<int>(kLaneSpeeds.size());
    vector<double> ego_config = {
        (double)kSpeedLimit, (double)num_lanes, (double)goal_s, (double)goal_lane, (double)kMaximumAcceleration};

    road.AddEgo(2, 0, ego_config);
    int timestep = 0;

    while (road.GetEgo().s_ <= kGoal[0])
    {
        timestep++;
        if (timestep > 100)
        {
            break;
        }
        road.Advance();
        road.Display(timestep);
    }
    Vehicle ego = road.GetEgo();
    if (ego.lane_ == kGoal[1])
    {
        cout << "You got to the goal in " << timestep << " seconds!" << endl;
        if (timestep > 35)
        {
            cout << "But it took too long to reach the goal. Go faster!" << endl;
        }
    }
    else
    {
        cout << "You missed the goal. You are in lane " << ego.lane_ << " instead of " << kGoal[1] << "." << endl;
    }
}
