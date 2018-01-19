///
/// @file
///

#ifndef PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H
#define PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H

#include "PathPlanner.h"
#include "spline.h"

class SimpleSplineBasedPlanner : public PathPlanner
{
  public:
    explicit SimpleSplineBasedPlanner(const HighwayMap& map, int starting_lane)
            : PathPlanner(map, starting_lane), target_speed_(0.0){};

    std::vector<CartesianPoint> GeneratePath(PathPlannerInput input) override;

  private:
    struct AnchorPoints
    {
        AnchorPoints(const CartesianPoint& reference_point,
                     const std::vector<CartesianPoint>& anchor_points)
                : reference_point(reference_point), anchor_points(anchor_points)
        {
        }

        CartesianPoint reference_point;
        std::vector<CartesianPoint> anchor_points;
    };

    void DecideDrivingPolicyForSpeedAndLane(PathPlannerInput input);
    void PrepareLaneChange();

    bool IsTooCloseToOtherCar(const PathPlannerInput& input) const;

    std::vector<CartesianPoint> ConvertPointsToLocalSystem(const std::vector<CartesianPoint>& new_path_anchor_points,
                                                           const CartesianPoint& local_reference_point) const;

    tk::spline MakeSplineFromAnchorPoints(const std::vector<CartesianPoint>& new_path_anchor_points) const;

    std::vector<CartesianPoint> GenerateNewPointsWithSpline(const tk::spline& new_path_spline,
                                                            int points_left_in_current_path) const;

    AnchorPoints GenerateAnchorPoints(const PathPlannerInput& input) const;

    double target_speed_;
    void ObeyRightLaneDrivingPolicy();
};

#endif  // PATH_PLANNING_SIMPLESPLINEBASEDPLANNER_H
