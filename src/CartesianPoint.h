///
/// @file
///

#ifndef PATH_PLANNING_CARTESIANPOINT_H
#define PATH_PLANNING_CARTESIANPOINT_H

#include <cmath>

struct CartesianPoint
{
    CartesianPoint() = default;
    CartesianPoint(double x0, double y0, double theta0 = 0.0) : x(x0), y(y0), theta(theta0) {}

    inline CartesianPoint ToLocal(const CartesianPoint& local_reference_point) const
    {
        double shift_x = x - local_reference_point.x;
        double shift_y = y - local_reference_point.y;

        return {(shift_x * cos(-local_reference_point.theta) - shift_y * sin(-local_reference_point.theta)),
                (shift_x * sin(-local_reference_point.theta) + shift_y * cos(-local_reference_point.theta))};
    };

    inline CartesianPoint ToGlobal(const CartesianPoint& local_reference_point) const
    {
        return {
            local_reference_point.x + (x * cos(local_reference_point.theta) - y * sin(local_reference_point.theta)),
            local_reference_point.y + (x * sin(local_reference_point.theta) + y * cos(local_reference_point.theta))};
    };

    double x;
    double y;
    double theta;
};

#endif  // PATH_PLANNING_CARTESIANPOINT_H
