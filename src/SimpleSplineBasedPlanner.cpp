///
/// @file
///

#include <cmath>
#include <iostream>
#include <vector>

#include "FsmList.h"
#include "SimpleSplineBasedPlanner.h"

std::vector<CartesianPoint> SimpleSplineBasedPlanner::GeneratePath(const PathPlannerInput& input)
{
    SendEvent(DataUpdate(input));

    target_speed_ = DrivingState::get_current_state()->GetTargetSpeed();
    target_lane_ = DrivingState::get_current_state()->GetTargetLane();

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
        auto previous_point = input.previous_path[input.previous_path.size() - 2];
        reference_point = input.previous_path.back();

        reference_point.theta = atan2(reference_point.y - previous_point.y, reference_point.x - previous_point.x);

        anchors.push_back(previous_point);
        anchors.push_back(reference_point);
    }

    const double kSplineInterpolationDistanceFactor = 50.0;
    for (auto& i : {1.25 * kSplineInterpolationDistanceFactor,
                    1.50 * kSplineInterpolationDistanceFactor,
                    1.75 * kSplineInterpolationDistanceFactor})
    {
        anchors.push_back(
            map_.FrenetToCartesian({input.frenet_location.s + i, FrenetPoint::LaneCenterDCoord(target_lane_)}));
    }

    return {reference_point, anchors};
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
    double kSimulatorRunloopPeriod = 0.02;
    const double kXAxisPlanningHorizon = 50.0;
    const int kMaxNumberOfPointsInPath = 50;

    const double path_end_point_x = kXAxisPlanningHorizon;
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

/// @deprecated: Now used in DrivingState class
void SimpleSplineBasedPlanner::DecideDrivingPolicyForSpeedAndLane(PathPlannerInput input)
{
    const double KDefaultAcceleration = 0.447;
    const double KMaxSpeed = 49.5;

    const auto kDistanceForFullBreak = 10.0;
    const auto kSpeedDifference = 10.0;

    auto is_too_close_and_distance = IsTooCloseToOtherCar(input);

    auto is_too_close_to_other_car = std::get<0>(is_too_close_and_distance);
    auto distance_to_other_car = std::get<1>(is_too_close_and_distance);
    auto other_car_speed = std::get<2>(is_too_close_and_distance);

    if (is_too_close_to_other_car)
    {
        // A simple deceleration controler which breaks harder the closer an other vehicle gets
        other_car_speed = MetersPerSecondToMph(other_car_speed);
        auto our_speed = input.speed;
        auto speed_difference = other_car_speed - our_speed;

        auto target_acceleration = (kDistanceForFullBreak / (2 * distance_to_other_car)) * KDefaultAcceleration;
        target_acceleration -= ((0.5 * speed_difference) / kSpeedDifference) * KDefaultAcceleration;
        target_acceleration = std::min(target_acceleration, KDefaultAcceleration);

        std::cout << "d_other | a_target | v_ego | v_other | v_diff | " << distance_to_other_car << " | "
                  << target_acceleration << " | " << our_speed << " | " << other_car_speed << " | " << speed_difference
                  << std::endl;

        target_speed_ -= target_acceleration;
    }
    else if (target_speed_ < KMaxSpeed)
    {
        target_speed_ += KDefaultAcceleration;
    }
}

/// @deprecated: Now used in DrivingState class
std::tuple<bool, double, double> SimpleSplineBasedPlanner::IsTooCloseToOtherCar(const PathPlannerInput& input) const
{
    const double kCriticalThresholdInMeters = 25.0;

    double ego_predicted_end_point_s =
        !input.previous_path.empty() ? input.path_endpoint_frenet.s : input.frenet_location.s;

    for (auto& other_car : input.other_cars)
    {
        if (other_car.IsInLane(target_lane_))
        {
            auto other_car_speed = other_car.Speed2DMagnitude();
            auto predicted_increase_of_s_wrt_our_car = input.previous_path.size() * other_car_speed;

            double other_car_predicted_s = other_car.frenet_location.s + predicted_increase_of_s_wrt_our_car;
            auto predicted_distance = other_car_predicted_s - ego_predicted_end_point_s;

            if ((other_car_predicted_s > ego_predicted_end_point_s) &&
                (predicted_distance < kCriticalThresholdInMeters))
            {
                return std::make_tuple(true, predicted_distance, other_car_speed);
            }
        }
    }
    return std::make_tuple(false, 0.0, 0.0);
}

/// @deprecated: Now used in DrivingState class
void SimpleSplineBasedPlanner::PrepareLaneChange(PathPlannerInput input)
{
    auto proposed_target_lane = PlanBehavior(target_lane_, input);
    std::cout << "Proposed lane to be changed to | " << proposed_target_lane << std::endl;

    static bool doing_lane_change = false;
    if (!doing_lane_change && proposed_target_lane != target_lane_)
    {
        doing_lane_change = true;
        target_lane_ = proposed_target_lane;
    }
    if (doing_lane_change && proposed_target_lane == target_lane_)
    {
        doing_lane_change = false;
    }
}

/// @deprecated: Now used in DrivingState class
void SimpleSplineBasedPlanner::ObeyRightLaneDrivingPolicy()
{
    /// @todo Check if save to change to right lane and if we need this at all
    target_lane_ = target_lane_;
}
