///
/// @file
///

#include "BehavioralPlanner.h"

enum class Lanes : int
{
    kRight = 2,
    kCenter = 1,
    kLeft = 0
};

BehavioralPlanner::BehavioralPlanner(int current_lane, double current_s) : lane_speeds_{kSpeedLimit, kSpeedLimit, kSpeedLimit}, road_(lane_speeds_)
{
    double goal_s = 1e9;
    int goal_lane = static_cast<int>(Lanes::kRight);
    vector<double> ego_config = {kSpeedLimit,
                                 static_cast<double>(lane_speeds_.size()),
                                 goal_s,
                                 static_cast<double>(goal_lane),
                                 kMaximumAcceleration};

    road_.AddEgo(current_lane, current_s, ego_config);
}

std::pair<double, double> BehavioralPlanner::Plan()
{
    double target_speed = 0;
    double target_lane = 0;

    road_.UpdateTraffic();
    road_.Advance();

    return std::make_pair(target_speed, target_lane);
}
