///
/// @file
///

#ifndef PATH_PLANNING_HIGHWAYMAP_H
#define PATH_PLANNING_HIGHWAYMAP_H

#include <cmath>
#include <string>
#include <vector>

#include "CartesianPoint.h"
#include "FrenetPoint.h"

class HighwayMap
{
  public:
    HighwayMap(const std::string& highway_map_csv_path);

    CartesianPoint FrenetToCartesian(const FrenetPoint& frenet_point) const;
    FrenetPoint CartesianToFrenet(const CartesianPoint& cartesian_point) const;

    int NextWaypoint(CartesianPoint current_vehicle_location) const;
    int ClosestWaypoint(CartesianPoint current_vehicle_location) const;

    static double EuclidDistance(CartesianPoint p1, CartesianPoint p2)
    {
        return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    }

  private:
    std::vector<double> map_points_x_;
    std::vector<double> map_points_y_;
    std::vector<double> map_points_s_;
    std::vector<double> map_points_dx_;
    std::vector<double> map_points_dy_;

    void ReadMapFromCsvFile(const std::string& highway_map_csv_path);
};

#endif  // PATH_PLANNING_HIGHWAYMAP_H
