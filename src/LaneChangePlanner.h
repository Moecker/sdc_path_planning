///
/// @file
///

#ifndef PATH_PLANNING_LANE_CHANGE_PLANNER_H
#define PATH_PLANNING_LANE_CHANGE_PLANNER_H

#include <vector>

#include "PathPlannerInput.h"

using std::vector;

// Check which lane is the best according to the rankings, then init lane change
void ChangeLaneCheck(int current_Lane,
                     double our_speed,
                     vector<vector<vector<double>>>& lane_lines,
                     vector<vector<int>>& close_cars,
                     vector<int>& lane_ranks);

// Identify the closest car ahead and behind of our car
void IdentClosestCars(vector<vector<vector<double>>>& lane_lines, vector<vector<int>>& close_cars);

// Give each lane a rank based on the score metrics / cost functions
void LaneRanking(int current_Lane,
                 vector<vector<vector<double>>>& lane_lines,
                 vector<vector<int>>& close_cars,
                 vector<int>& lane_ranks);

void BehaviorPlanner(int current_lane, PathPlannerInput input);

#endif  // PATH_PLANNING_LANE_CHANGE_PLANNER_H