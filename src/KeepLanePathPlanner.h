///
/// @file
///

#ifndef PATH_PLANNING_KEEPLANEPATHPLANNER_H
#define PATH_PLANNING_KEEPLANEPATHPLANNER_H

#include "PathPlanner.h"

class KeepLanePathPlanner : public PathPlanner
{
  public:
    explicit KeepLanePathPlanner(const HighwayMap& map, int starting_lane) : PathPlanner(map, starting_lane){};
    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;
};

#endif  // PATH_PLANNING_KEEPLANEPATHPLANNER_H
