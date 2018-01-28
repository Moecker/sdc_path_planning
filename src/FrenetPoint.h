///
/// @file
///

#ifndef PATH_PLANNING_FRENETPOINT_H
#define PATH_PLANNING_FRENETPOINT_H

const double kLaneWidthInD = 4.0;

struct FrenetPoint
{
    FrenetPoint() = default;
    FrenetPoint(double s, double d) : s(s), d(d) {}

    inline static double LaneCenterDCoord(int lane_number)
    {
        return kLaneWidthInD * lane_number + kLaneWidthInD / 2.0;
    };

    inline bool IsInLane(int lane_number) const
    {
        return (d < (kLaneWidthInD * (lane_number + 1))) && (d > (kLaneWidthInD * lane_number));
    }

    double s = 0.0;
    double d = 0.0;
};

#endif  // PATH_PLANNING_FRENETPOINT_H
