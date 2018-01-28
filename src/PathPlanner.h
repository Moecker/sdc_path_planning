///
/// @file
///

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "HighwayMap.h"
#include "PathPlannerInput.h"

class PathPlanner
{
  public:
    explicit PathPlanner(const HighwayMap& map, int starting_lane) : map_(map), target_lane_(starting_lane) {}
    virtual std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) = 0;

  protected:
    const HighwayMap& map_;
    int target_lane_ = 0;
};

#endif  // PATH_PLANNING_PATHPLANNER_H
