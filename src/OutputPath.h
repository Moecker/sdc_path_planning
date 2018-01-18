///
/// @file
///

#ifndef PATH_PLANNING_OUTPUTPATH_H
#define PATH_PLANNING_OUTPUTPATH_H

#include <CartesianPoint.h>
#include <vector>

struct OutputPath
{
    std::vector<double> PathX;
    std::vector<double> PathY;

    std::vector<CartesianPoint> Path;
};

#endif  // PATH_PLANNING_OUTPUTPATH_H
