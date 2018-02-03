///
/// @file
/// @brief Note: Deprecated, as we use the DrivingStateMachine for behavior planning
///

#include "BehavioralPlanner.h"

enum class Lanes : int
{
    kRight = 2,
    kCenter = 1,
    kLeft = 0
};

/// @deprecated: Now used in DrivingState class
BehavioralPlanner::BehavioralPlanner(int current_lane, double current_s)
        : lane_speeds_{kSpeedLimit, kSpeedLimit, kSpeedLimit}, road_(lane_speeds_)
{
    double goal_s = 1e9;
    int goal_lane = static_cast<int>(Lanes::kRight);
    vector<double> ego_config = {kSpeedLimit,
                                 static_cast<double>(lane_speeds_.size()),
                                 goal_s,
                                 static_cast<double>(goal_lane),
                                 kMaximumAcceleration};
}

/// @deprecated: Now used in DrivingState class
std::pair<double, double> BehavioralPlanner::Plan(PathPlannerInput input)
{
    road_.UpdateTraffic(input.other_cars);
    road_.UpdateEgo(input.frenet_location, input.lane, input.speed);

    road_.AdvanceNew();

    double target_speed = road_.ego_.v_;
    double target_lane = road_.ego_.lane_;

    return std::make_pair(target_speed, target_lane);
}
