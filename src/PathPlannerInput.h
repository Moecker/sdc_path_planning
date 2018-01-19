///
/// @file
///

#ifndef PATH_PLANNING_PATHPLANNERINPUT_H
#define PATH_PLANNING_PATHPLANNERINPUT_H

#include <vector>

#include "CartesianPoint.h"
#include "FrenetPoint.h"
#include "OtherCar.h"

struct PathPlannerInput
{
    CartesianPoint cartesian_location;

    FrenetPoint frenet_location;
    FrenetPoint path_endpoint_frenet;

    double speed;

    std::vector<CartesianPoint> previous_path;

    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;

    std::vector<OtherCar> other_cars;
};

#endif  // PATH_PLANNING_PATHPLANNERINPUT_H
