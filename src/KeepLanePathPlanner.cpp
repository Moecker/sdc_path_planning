///
/// @file
///

#include "KeepLanePathPlanner.h"

std::vector<CartesianPoint> KeepLanePathPlanner::GeneratePath(PathPlannerInput input)
{
    std::vector<CartesianPoint> output_path;
    double dist_inc = 0.3;

    for (int i = 0; i < 50; i++)
    {
        double next_s = input.frenet_location.s + (i + 1) * dist_inc;
        double next_d = 6;

        output_path.push_back(map_.FrenetToCartesian({next_s, next_d}));
    }
    return output_path;
}
