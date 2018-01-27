///
/// @file
///

#include <algorithm>
#include <vector>

#include <Road.h>
#include <PathPlannerInput.h>

#ifndef BEHAVIORAL_PLANNING_H
#define BEHAVIORAL_PLANNING_H

class BehavioralPlanner
{
  public:
    const double kMaximumAcceleration = 0.447;
    const double kSpeedLimit = 49.5;

    BehavioralPlanner(int current_lane, double current_s);
    std::pair<double, double> Plan(PathPlannerInput input);

  private:
    std::vector<double> lane_speeds_;
    Road road_;
};

#endif  // BEHAVIORAL_PLANNING_H
