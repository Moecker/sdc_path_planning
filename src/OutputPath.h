///
/// @file
///

#ifndef PATH_PLANNING_OUTPUTPATH_H
#define PATH_PLANNING_OUTPUTPATH_H

#include <CartesianPoint.h>
#include <vector>

struct OutputPath
{
    std::vector<double> path_x;
    std::vector<double> path_y;

    std::vector<CartesianPoint> path;
};

#endif  // PATH_PLANNING_OUTPUTPATH_H
