///
/// @file
///

#include <cmath>
#include <iostream>
#include <vector>

#include "SimpleSplineBasedPlanner.h"

double Deg2Rad(double x)
{
    return x * M_PI / 180;
}
double MphToMetersPerSecond(double mph_value)
{
    return mph_value / 2.24;
}

static const double KDefaultAcceleration = 0.224;
static const double KMaxSpeed = 49.5;
static const double kCriticalThresholdInMeters = 30.0;
static const double kSimulatorRunloopPeriod = 0.02;

static const int kXAxisPlanningHorizon = 30;
static const int kMaxNumberOfPointsInPath = 50;
static const int kLeftmostLaneNumber = 0;

std::vector<CartesianPoint> SimpleSplineBasedPlanner::GeneratePath(PathPlannerInput input)
{
    DecideDrivingPolicyForSpeedAndLane(input);

    auto anchors_cartesian = GenerateAnchorPoints(input);
    auto anchors_local = ConvertPointsToLocalSystem(anchors_cartesian.anchor_points, anchors_cartesian.reference_point);

    auto spline = MakeSplineFromAnchorPoints(anchors_local);
    auto new_path_points = GenerateNewPointsWithSpline(spline, static_cast<int>(input.previous_path.size()));

    std::vector<CartesianPoint> output_path = {input.previous_path};
    for (auto& p : new_path_points)
    {
        output_path.push_back(p.ToGlobal(anchors_cartesian.reference_point));
    }

    return output_path;
}

void SimpleSplineBasedPlanner::DecideDrivingPolicyForSpeedAndLane(PathPlannerInput input)
{
    /// @todo Decide what is the best lane to be in a couple of seconds
    ObeyRightLaneDrivingPolicy();
    if (IsTooCloseToOtherCar(input))
    {
        target_speed_ -= KDefaultAcceleration;
        PrepareLaneChange();
    }
    else if (target_speed_ < KMaxSpeed)
    {
        target_speed_ += KDefaultAcceleration;
    }
}

bool SimpleSplineBasedPlanner::IsTooCloseToOtherCar(const PathPlannerInput& input) const
{
    double ego_predicted_end_point_s =
        !input.previous_path.empty() ? input.path_endpoint_frenet.s : input.frenet_location.s;

    for (auto& other_car : input.other_cars)
    {
        if (other_car.IsInLane(target_lane_))
        {
            auto predicted_increase_of_s_wrt_our_car =
                input.previous_path.size() * kSimulatorRunloopPeriod * other_car.Speed2DMagnitude() * 0.447;

            double other_car_predicted_s = other_car.frenet_location.s + predicted_increase_of_s_wrt_our_car;
            if (other_car_predicted_s > ego_predicted_end_point_s &&
                (other_car_predicted_s - ego_predicted_end_point_s) < kCriticalThresholdInMeters)
                return true;
        }
    }
    return false;
}

SimpleSplineBasedPlanner::AnchorPoints SimpleSplineBasedPlanner::GenerateAnchorPoints(
    const PathPlannerInput& input) const
{
    CartesianPoint reference_point{};

    std::vector<CartesianPoint> anchors;
    if (input.previous_path.empty() || input.previous_path.size() == 1)
    {
        anchors.push_back({input.cartesian_location.x - cos(input.cartesian_location.theta),
                           input.cartesian_location.y - sin(input.cartesian_location.theta)});

        reference_point = input.cartesian_location;
        reference_point.theta = Deg2Rad(reference_point.theta);
        anchors.push_back(reference_point);
    }
    else
    {
        reference_point = input.previous_path.back();
        auto previous_point = input.previous_path[input.previous_path.size() - 2];

        reference_point.theta = atan2(reference_point.y - previous_point.y, reference_point.x - previous_point.x);

        anchors.push_back(previous_point);
        anchors.push_back(reference_point);
    }

    double distance_at{0.0};
    const double kSplineInterpolationDistanceFactor = 30.0;
    for (auto& i : {distance_at = kSplineInterpolationDistanceFactor,
                   distance_at = 2 * kSplineInterpolationDistanceFactor,
                   distance_at = 3 * kSplineInterpolationDistanceFactor})
    {
        anchors.push_back(
            map_.FrenetToCartesian({input.frenet_location.s + i, FrenetPoint::LaneCenterDCoord(target_lane_)}));
    }

    return {reference_point, anchors};
}

void SimpleSplineBasedPlanner::ObeyRightLaneDrivingPolicy()
{
    /// @todo Check if save to change to right lane
    target_lane_ = target_lane_;
}

void SimpleSplineBasedPlanner::PrepareLaneChange()
{
    /// @todo Check if save to change to left lane
    target_lane_ = kLeftmostLaneNumber;
}

std::vector<CartesianPoint> SimpleSplineBasedPlanner::ConvertPointsToLocalSystem(
    const std::vector<CartesianPoint>& anchor_points_global,
    const CartesianPoint& local_reference_point) const
{
    std::vector<CartesianPoint> anchor_points_local;
    for (auto& p : anchor_points_global)
    {
        anchor_points_local.push_back(p.ToLocal(local_reference_point));
    }
    return anchor_points_local;
}

std::vector<CartesianPoint> SimpleSplineBasedPlanner::GenerateNewPointsWithSpline(const tk::spline& new_path_spline,
                                                                                  int points_left_in_current_path) const
{
    const double path_end_point_x = 30.0;
    double path_end_point_y = new_path_spline(path_end_point_x);
    double path_length = sqrt(path_end_point_x * path_end_point_x + path_end_point_y * path_end_point_y);

    std::vector<CartesianPoint> path_points;

    double previous_x = 0.0;
    double number_of_points = path_length / (kSimulatorRunloopPeriod * MphToMetersPerSecond(target_speed_));
    double x_axis_step = kXAxisPlanningHorizon / number_of_points;

    for (int i = 1; i <= kMaxNumberOfPointsInPath - points_left_in_current_path; i++)
    {
        double x = previous_x + x_axis_step;
        double y = new_path_spline(x);

        previous_x = x;
        path_points.emplace_back(x, y);
    }
    return path_points;
}

tk::spline SimpleSplineBasedPlanner::MakeSplineFromAnchorPoints(
    const std::vector<CartesianPoint>& new_path_anchor_points) const
{
    std::vector<double> new_path_anchors_x;
    std::vector<double> new_path_anchors_y;
    for (auto& p : new_path_anchor_points)
    {
        new_path_anchors_x.push_back(p.x);
        new_path_anchors_y.push_back(p.y);
    }
    tk::spline spline;
    spline.set_points(new_path_anchors_x, new_path_anchors_y);
    return spline;
}
