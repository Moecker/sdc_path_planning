#include "DrivingStateMachine.h"
#include "SimpleSplineBasedPlanner.h"

#include <algorithm>
#include <fsmlist.hpp>
#include <iostream>
#include <tinyfsm.hpp>
#include <typeinfo>

using namespace std;

static const double KDefaultAcceleration = 0.70;
static const double KMaxSpeed = 49.0;
static const double kCriticalThresholdInMeters = 40.0;
static const double kSimulatorRunloopPeriod = 0.02;
static const double kXAxisPlanningHorizon = 50.0;

static const int kMaxNumberOfPointsInPath = 50;
static const int kLeftmostLaneNumber = 0;

double Deg2Rad(double x)
{
    return x * M_PI / 180;
}
double MphToMetersPerSecond(double mph_value)
{
    return mph_value / 2.24;
}
double MetersPerSecondToMph(double mps_value)
{
    return mps_value * 2.24;
}

vector<vector<OtherCar>> GetLaneRelatedOtherCars(const vector<OtherCar>& other_cars)
{
    vector<OtherCar> left, center, right;
    for (auto car : other_cars)
    {
        switch (car.lane)
        {
            case 0:
                left.emplace_back(car);
                break;
            case 1:
                center.emplace_back(car);
                break;
            case 2:
                right.emplace_back(car);
                break;
        }
    }
    vector<vector<OtherCar>> ret;
    ret.push_back(left);
    ret.push_back(center);
    ret.push_back(right);
    return ret;
}

std::tuple<bool, double, double> IsTooCloseToOtherCar(const PathPlannerInput& input, int target_lane)
{
    for (auto& other_car : input.other_cars)
    {
        if (other_car.IsInLane(target_lane))
        {
            auto other_car_speed = other_car.Speed2DMagnitude();
            auto predicted_increase_of_s_other_car = kSimulatorRunloopPeriod * other_car_speed;
            double other_car_predicted_s = other_car.frenet_location.s + predicted_increase_of_s_other_car;

            auto predicted_increase_of_s_ego = kSimulatorRunloopPeriod * input.speed;
            double ego_predicted_s = input.frenet_location.s + predicted_increase_of_s_ego;

            auto predicted_distance = other_car_predicted_s - ego_predicted_s;

            if ((other_car_predicted_s > ego_predicted_s) && (predicted_distance < kCriticalThresholdInMeters))
            {
                return std::make_tuple(true, predicted_distance, other_car_speed);
            }
        }
    }
    return std::make_tuple(false, 0.0, 0.0);
}

std::pair<bool, double> SimpleKeepLaneControler(const PathPlannerInput& input, double target_speed, int target_lane)
{
    auto is_too_close_and_distance = IsTooCloseToOtherCar(input, target_lane);
    bool is_too_close_to_other_car = std::get<0>(is_too_close_and_distance);

    double target_speed_out = target_speed;

    if (is_too_close_to_other_car)
    {
        target_speed_out -= KDefaultAcceleration;
    }
    else if (target_speed < KMaxSpeed)
    {
        target_speed_out += KDefaultAcceleration;
    }

    return std::make_pair(is_too_close_to_other_car, target_speed_out);
}

std::pair<bool, double> BasicKeepLaneControler(const PathPlannerInput& input, double target_speed, int target_lane)
{
    const auto kDistanceForFullBreak = 10.0;
    const auto kSpeedDifference = 10.0;

    auto is_too_close_and_distance = IsTooCloseToOtherCar(input, target_lane);
    bool is_too_close_to_other_car = std::get<0>(is_too_close_and_distance);

    double target_speed_out = target_speed;

    if (is_too_close_to_other_car)
    {
        auto distance_to_other_car = std::get<1>(is_too_close_and_distance);
        auto other_car_speed = std::get<2>(is_too_close_and_distance);

        // A simple deceleration controler which breaks harder the closer an other vehicle gets
        other_car_speed = MetersPerSecondToMph(other_car_speed);
        auto our_speed = input.speed;
        auto speed_difference = other_car_speed - our_speed;

        auto target_acceleration = (kDistanceForFullBreak / (3 * distance_to_other_car)) * KDefaultAcceleration;
        target_acceleration -= ((0.3 * speed_difference) / kSpeedDifference) * KDefaultAcceleration;
        target_acceleration = std::min(target_acceleration, KDefaultAcceleration);

        target_speed_out -= target_acceleration;
    }
    else if (target_speed < KMaxSpeed)
    {
        target_speed_out += KDefaultAcceleration;
    }

    return std::make_pair(is_too_close_to_other_car, target_speed_out);
}

// Motor states
class KeepingLane : public DrivingState
{
  private:
    void entry() override { cout << "Keeping Lane" << endl; }

    void react(DataUpdate const& update) override { DecideDrivingPolicyForSpeedAndLane(update.payload); }

    double GetAverageSpeed(int lane, vector<OtherCar> other_cars)
    {
        auto average_speed = 0.0;
        auto lr_cars = GetLaneRelatedOtherCars(other_cars);
        if (lane <= 2 && lane >= 0)
        {
            auto lane_cars = lr_cars[lane];
            double acc_speed =
                std::accumulate(lane_cars.begin(), lane_cars.end(), 0.0, [](double result, OtherCar car2) {
                    return result + car2.Speed2DMagnitude();
                });
            average_speed = acc_speed / lane_cars.size();
        }

        return average_speed;
    }

    void DecideDrivingPolicyForSpeedAndLane(const PathPlannerInput& input)
    {
        auto lane_controller_output = BasicKeepLaneControler(input, target_speed_, target_lane_);
        auto other_car_too_close = lane_controller_output.first;
        if (other_car_too_close)
        {
            auto av_speed_current = GetAverageSpeed(target_lane_, input.other_cars);
            /// @todo Fixme
            av_speed_current = 0.0;
            auto av_speed_left = GetAverageSpeed(target_lane_ - 1, input.other_cars);
            auto av_speed_right = GetAverageSpeed(target_lane_ + 1, input.other_cars);

            if (av_speed_current < max(av_speed_left, av_speed_right))
            {
                if (av_speed_left > av_speed_right)
                {
                    SendEvent(PrepareLaneChangeLeftIntent());
                }
                else
                {
                    SendEvent(PrepareLaneChangeRightIntent());
                }
            }
        }

        auto target_speed = lane_controller_output.second;
        target_speed_ = target_speed;
    }
};

class ChangingLaneRight : public DrivingState
{
    void entry() override
    {
        cout << "Changing lane right" << endl;
        target_lane_ += 1;
    };

    void react(DataUpdate const& update) override
    {
        if (target_lane_ == update.payload.lane)
        {
            SendEvent(LaneChangeCompleted());
        }
    }
};

class ChangingLaneLeft : public DrivingState
{
    void entry() override
    {
        cout << "Changing lane left" << endl;
        target_lane_ -= 1;
    };

    void react(DataUpdate const& update) override
    {
        if (target_lane_ == update.payload.lane)
        {
            SendEvent(LaneChangeCompleted());
        }
    }
};

double PredictDistanceInGivenSeconds(double pred_seconds, double current_s, double speed, OtherCar& other_car)
{
    auto predicted_other_car =
        other_car.frenet_location.s + pred_seconds * MphToMetersPerSecond(other_car.Speed2DMagnitude());
    auto predicted_ego = current_s + pred_seconds * MphToMetersPerSecond(speed);
    return abs(predicted_other_car - predicted_ego);
}

struct greater_frenet
{
    template <class T>
    bool operator()(T const& a, T const& b) const
    {
        return a.frenet_location.s > b.frenet_location.s;
    }
};

struct smaller_frenet
{
    template <class T>
    bool operator()(T const& a, T const& b) const
    {
        return a.frenet_location.s < b.frenet_location.s;
    }
};

bool IsSafeToChangeLane(double current_s, double speed, vector<OtherCar>& other_cars)
{
    auto kSafetyDistance = 15.0;
    std::sort(other_cars.begin(), other_cars.end(), smaller_frenet());

    cout << "Sorted cars s: ";
    std::for_each(other_cars.begin(), other_cars.end(), [](OtherCar car) { cout << car.frenet_location.s << " | "; });
    cout << endl;
    std::for_each(other_cars.begin(), other_cars.end(), [](OtherCar car) { cout << car.lane << " | "; });
    cout << endl;
    std::for_each(other_cars.begin(), other_cars.end(), [](OtherCar car) { cout << car.frenet_location.d << " | "; });
    cout << endl;

    cout << "Current s: " << current_s << endl;

    auto first_car_ahead = std::stable_partition(other_cars.begin(), other_cars.end(), [current_s](OtherCar car) {
        return (car.frenet_location.s > current_s);
    });

    auto closest_car_ahead = std::max_element(other_cars.begin(), first_car_ahead, smaller_frenet());
    auto closest_car_behind = std::max_element(first_car_ahead, other_cars.end(), smaller_frenet());

    /// @todo Use min_element instead of find_if!

    bool ahead_ok = true;
    bool behind_ok = true;
    if (closest_car_behind != other_cars.end() && closest_car_behind >= other_cars.begin())
    {
        cout << "Closest car behind s: " << closest_car_behind->frenet_location.s << " | ";
        auto distance = abs(current_s - closest_car_behind->frenet_location.s);
        distance = PredictDistanceInGivenSeconds(1.0, current_s, speed, *closest_car_behind);
        /// @todo Fixme
        cout << "dist_behind: " << distance << " | ";
        behind_ok = (distance > kSafetyDistance);
    }
    if (closest_car_ahead != other_cars.end() && closest_car_behind >= other_cars.begin())
    {
        cout << "Closest car ahead s: " << closest_car_ahead->frenet_location.s << " | ";
        auto distance = abs(current_s - closest_car_ahead->frenet_location.s);
        distance = PredictDistanceInGivenSeconds(1.0, current_s, speed, *closest_car_ahead);
        /// @todo Fixme
        cout << "dist_ahead: " << distance << " | ";
        ahead_ok = (distance > kSafetyDistance);
    }
    cout << endl;
    // return false;
    return ahead_ok && behind_ok;
}

bool IsLaneChangeFeasible(const PathPlannerInput& input, const int for_lane)
{
    auto lane_related_other_cars = GetLaneRelatedOtherCars(input.other_cars);
    auto is_safe = IsSafeToChangeLane(input.frenet_location.s, input.speed, lane_related_other_cars[for_lane]);
    return is_safe;
}

void DrivingState::DefaultPrepareLaneChangeLogic(DataUpdate const& update, int final_lane)
{
    /// @todo Fixme
    if (true)  // if (update.payload.speed > (KMaxSpeed - 15.0))
    {
        auto is_feasible = IsLaneChangeFeasible(update.payload, final_lane);
        if (is_feasible)
        {
            if (final_lane > target_lane_)
                SendEvent(ChangeLaneRightIntent());
            else
                SendEvent(ChangeLaneLeftIntent());
        }
        else
        {
            auto lane_controller_output = BasicKeepLaneControler(update.payload, target_speed_, target_lane_);
            auto target_speed = lane_controller_output.second;
            target_speed_ = target_speed;
        }
    }
    else
    {
        SendEvent(AbortLaneChange());
    }
}

class PreparingLaneChangeRight : public DrivingState
{
    void entry() override { cout << "Preparing to change lane right" << endl; }

    void react(DataUpdate const& update) override
    {
        if (update.payload.lane < 2)
        {
            DefaultPrepareLaneChangeLogic(update, target_lane_ + 1);
        }
    }
};

class PreparingLaneChangeLeft : public DrivingState
{
    void entry() override { cout << "Preparing to change lane left" << endl; }

    void react(DataUpdate const& update) override
    {
        if (update.payload.lane > 0)
        {
            DefaultPrepareLaneChangeLogic(update, target_lane_ - 1);
        }
    }
};

// Base State: default implementations
void DrivingState::react(ChangeLaneRightIntent const&)
{
    transit<ChangingLaneRight>();
}

void DrivingState::react(ChangeLaneLeftIntent const&)
{
    transit<ChangingLaneLeft>();
}

void DrivingState::react(PrepareLaneChangeRightIntent const&)
{
    transit<PreparingLaneChangeRight>();
}

void DrivingState::react(PrepareLaneChangeLeftIntent const&)
{
    transit<PreparingLaneChangeLeft>();
}

void DrivingState::react(LaneChangeCompleted const&)
{
    transit<KeepingLane>();
}

void DrivingState::react(AbortLaneChange const&)
{
    transit<KeepingLane>();
}

void DrivingState::react(DataUpdate const& update)
{
    DrivingState::input_ = update.payload;
}

// Initialization of static members
PathPlannerInput DrivingState::input_ = PathPlannerInput();
int DrivingState::target_lane_ = 1;
double DrivingState::target_speed_ = 0.0;

// Initial state definition
FSM_INITIAL_STATE(DrivingState, KeepingLane)
