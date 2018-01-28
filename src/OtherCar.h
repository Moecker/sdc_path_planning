///
/// @file
///

#ifndef PATH_PLANNING_OTHERCAR_H
#define PATH_PLANNING_OTHERCAR_H

#include <cmath>

#include "CartesianPoint.h"
#include "FrenetPoint.h"

struct OtherCar
{
    inline double Speed2DMagnitude() const { return sqrt(x_axis_speed * x_axis_speed + y_axis_speed * y_axis_speed); }
    inline bool IsInLane(int lane_number) const { return frenet_location.IsInLane(lane_number); }

    CartesianPoint cartesian_location;
    double x_axis_speed = 0.0;
    double y_axis_speed = 0.0;
    FrenetPoint frenet_location;
    int lane = 0;
};

#endif  // PATH_PLANNING_OTHERCAR_H
