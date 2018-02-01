///
/// @file
///

#include <fstream>
#include <sstream>
#include <thread>

#include <uWS/uWS.h>
#include "HighwayMap.h"

HighwayMap::HighwayMap(const std::string& highway_map_csv_path)
{
    ReadMapFromCsvFile(highway_map_csv_path);
}

CartesianPoint HighwayMap::FrenetToCartesian(const FrenetPoint& frenet_point) const
{
    int previous_point = -1;

    while (frenet_point.s > map_points_s_[previous_point + 1] && (previous_point < (int)(map_points_s_.size() - 1)))
    {
        previous_point++;
    }

    auto wp2 = static_cast<int>((previous_point + 1) % map_points_x_.size());

    double heading = atan2((map_points_y_[wp2] - map_points_y_[previous_point]),
                           (map_points_x_[wp2] - map_points_x_[previous_point]));

    // the x,y,s along the segment
    double seg_s = (frenet_point.s - map_points_s_[previous_point]);

    double seg_x = map_points_x_[previous_point] + seg_s * cos(heading);
    double seg_y = map_points_y_[previous_point] + seg_s * sin(heading);

    double perp_heading = heading - M_PI_2;

    double x = seg_x + frenet_point.d * cos(perp_heading);
    double y = seg_y + frenet_point.d * sin(perp_heading);

    return {x, y};
}

FrenetPoint HighwayMap::CartesianToFrenet(const CartesianPoint& cartesian_point) const
{
    int next_wp = NextWaypoint(cartesian_point);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = map_points_x_.size() - 1;
    }

    double n_x = map_points_x_[next_wp] - map_points_x_[prev_wp];
    double n_y = map_points_y_[next_wp] - map_points_y_[prev_wp];
    double x_x = cartesian_point.x - map_points_x_[prev_wp];
    double x_y = cartesian_point.y - map_points_y_[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = EuclidDistance(cartesian_point, {proj_x, proj_y});

    double center_x = 1000 - map_points_x_[prev_wp];
    double center_y = 2000 - map_points_y_[prev_wp];
    double centerToPos = EuclidDistance({center_x, center_y}, {x_x, x_y});
    double centerToRef = EuclidDistance({center_x, center_y}, {proj_x, proj_y});

    // see if d value is positive or negative by comparing it to a center point
    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++)
    {
        frenet_s += EuclidDistance({map_points_x_[i], map_points_y_[i]}, {map_points_x_[i + 1], map_points_y_[i + 1]});
    }

    frenet_s += EuclidDistance({0, 0}, {proj_x, proj_y});

    return {frenet_s, frenet_d};
}

int HighwayMap::NextWaypoint(CartesianPoint current_vehicle_location) const
{
    int closest_waypoint = ClosestWaypoint(current_vehicle_location);

    double map_x = map_points_x_[closest_waypoint];
    double map_y = map_points_y_[closest_waypoint];

    double heading = atan2((map_y - current_vehicle_location.y), (map_x - current_vehicle_location.x));

    double angle = std::abs(current_vehicle_location.theta - heading);

    if (angle > M_PI_4)
    {
        closest_waypoint++;
    }

    return closest_waypoint;
}

int HighwayMap::ClosestWaypoint(CartesianPoint current_vehicle_location) const
{
    double closest_length = 100000;  // large number
    int closest_waypoint = 0;

    for (int i = 0; i < map_points_x_.size(); i++)
    {
        double map_x = map_points_x_[i];
        double map_y = map_points_y_[i];
        double dist = EuclidDistance(current_vehicle_location, {map_x, map_y});
        if (dist < closest_length)
        {
            closest_length = dist;
            closest_waypoint = i;
        }
    }

    return closest_waypoint;
}

void HighwayMap::ReadMapFromCsvFile(const std::string& highway_map_csv_path)
{
    std::ifstream in_map_(highway_map_csv_path.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x, y, s, d_x, d_y;
        iss >> x >> y >> s >> d_x >> d_y;

        map_points_x_.push_back(x);
        map_points_y_.push_back(y);
        map_points_s_.push_back(s);
        map_points_dx_.push_back(d_x);
        map_points_dy_.push_back(d_y);
    }
}
