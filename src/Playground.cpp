#include <fsmlist.hpp>
#include "Road.h"

struct Base
{
    virtual void ReimplementMe(int a) {}
};

struct Derived : public Base
{
    void ReimplementMe(int a) override {}
};

void RunBehaviorPlanner()
{
    using namespace std;

    // impacts default behavior for most states
    int SPEED_LIMIT = 10;

    // all traffic in lane (besides ego) follow these speeds
    vector<int> LANE_SPEEDS = {6, 7, 8, 9};

    // Number of available "cells" which should have traffic
    double TRAFFIC_DENSITY = 0.15;

    // At each timestep, ego can set acceleration to value between
    int MAX_ACCEL = 2;

    // s value and lane number of goal.
    vector<int> GOAL = {300, 0};

    // These affect the visualization
    int AMOUNT_OF_ROAD_VISIBLE = 40;

    Road road = Road(TRAFFIC_DENSITY, LANE_SPEEDS);

    road.kUpdateWidth = AMOUNT_OF_ROAD_VISIBLE;

    road.populate_traffic();

    int goal_s = GOAL[0];
    int goal_lane = GOAL[1];

    int num_lanes = static_cast<int>(LANE_SPEEDS.size());
    vector<int> ego_config = {SPEED_LIMIT, num_lanes, goal_s, goal_lane, MAX_ACCEL};

    road.add_ego(2, 0, ego_config);
    int timestep = 0;

    while (road.get_ego().s <= GOAL[0])
    {
        timestep++;
        if (timestep > 100)
        {
            break;
        }
        road.advance();
        road.display(timestep);
    }
    Vehicle ego = road.get_ego();
    if (ego.lane == GOAL[1])
    {
        cout << "You got to the goal in " << timestep << " seconds!" << endl;
        if (timestep > 35)
        {
            cout << "But it took too long to reach the goal. Go faster!" << endl;
        }
    }
    else
    {
        cout << "You missed the goal. You are in lane " << ego.lane << " instead of " << GOAL[1] << "." << endl;
    }
}

void StateMachineTest()
{
    MotorDown motor_down;
    MotorUp motor_up;

    SendEvent(motor_down);
    SendEvent(motor_up);
    SendEvent(motor_down);
}
