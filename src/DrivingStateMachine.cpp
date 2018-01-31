///
/// @file
///

#include "DrivingStateMachine.h"
#include "SimpleSplineBasedPlanner.h"

#include <pid/pid.h>
#include <algorithm>
#include <fsmlist.hpp>
#include <future>
#include <iostream>
#include <numeric>
#include <tinyfsm.hpp>
#include <typeinfo>

enum Lanes
{
    kRight = 2,
    kCenter = 1,
    kLeft = 0
};

static const double KDefaultAcceleration = 0.70;
static const double KMaxSpeed = 49.0;
static const double kCriticalThresholdInMeters = 40.0;
static const double kSimulatorRunloopPeriod = 0.02;
static const double kXAxisPlanningHorizon = 50.0;

static const int kMaxNumberOfPointsInPath = 50;

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

namespace
{

typedef vector<OtherCar>::const_iterator OtherCarIterator;
std::pair<OtherCarIterator, OtherCarIterator> FindClosestCarsBehindAndAhead(double current_s,
                                                                            const vector<OtherCar>& other_cars)
{
    OtherCarIterator car_behind = other_cars.end();
    OtherCarIterator car_ahead = other_cars.end();
    for (OtherCarIterator car = other_cars.begin(); car != other_cars.end(); ++car)
    {
        if (current_s > car->frenet_location.s)
        {
            car_behind = car;
        }
        else
        {
            car_ahead = car;
            break;
        }
    }
    return std::make_pair(car_behind, car_ahead);
}

vector<vector<OtherCar>> GetLaneRelatedOtherCars(const vector<OtherCar>& other_cars)
{
    vector<OtherCar> left, center, right;
    for (auto car : other_cars)
    {
        switch (car.lane)
        {
            case kLeft:
                left.emplace_back(car);
                break;
            case kCenter:
                center.emplace_back(car);
                break;
            case kRight:
                right.emplace_back(car);
                break;
        }
    }
    vector<vector<OtherCar>> vector_of_cars;
    vector_of_cars.push_back(left);
    vector_of_cars.push_back(center);
    vector_of_cars.push_back(right);
    return vector_of_cars;
}

std::tuple<bool, double, double> IsTooCloseToOtherCar(const PathPlannerInput& input, int target_lane)
{
    for (const auto& other_car : input.other_cars)
    {
        if (other_car.IsInLane(target_lane))
        {
            auto other_car_speed = other_car.Speed2DMagnitude();
            auto predicted_increase_of_s_other_car = kSimulatorRunloopPeriod * other_car_speed;
            auto other_car_predicted_s = other_car.frenet_location.s + predicted_increase_of_s_other_car;

            auto predicted_increase_of_s_ego = kSimulatorRunloopPeriod * input.speed;
            auto ego_predicted_s = input.frenet_location.s + predicted_increase_of_s_ego;

            auto predicted_distance = other_car_predicted_s - ego_predicted_s;

            if ((other_car_predicted_s > ego_predicted_s) && (predicted_distance < kCriticalThresholdInMeters))
            {
                return std::make_tuple(true, predicted_distance, other_car_speed);
            }
        }
    }
    return std::make_tuple(false, 0.0, 0.0);
}

std::pair<bool, double> PIDKeepLaneControler(const PathPlannerInput& input, double target_speed, int target_lane)
{
    static PID pid = PID(0.1, 100, -100, kSimulatorRunloopPeriod, KMaxSpeed, KDefaultAcceleration);
    auto is_too_close_and_distance = IsTooCloseToOtherCar(input, target_lane);

    bool is_too_close_to_other_car = std::get<0>(is_too_close_and_distance);
    auto other_car_speed = std::get<2>(is_too_close_and_distance);

    double new_speed = pid.calculate(other_car_speed, target_speed);
    double target_speed_out = new_speed;

    return std::make_pair(is_too_close_to_other_car, target_speed_out);
}

std::pair<bool, double> SimpleKeepLaneControler(const PathPlannerInput& input, double target_speed, int target_lane)
{
    auto is_too_close_and_distance = IsTooCloseToOtherCar(input, target_lane);
    auto is_too_close_to_other_car = std::get<0>(is_too_close_and_distance);

    auto target_speed_out = target_speed;

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
    auto is_too_close_to_other_car = std::get<0>(is_too_close_and_distance);

    auto target_speed_out = target_speed;

    if (is_too_close_to_other_car)
    {
        auto distance_to_other_car = std::get<1>(is_too_close_and_distance);
        auto other_car_speed = std::get<2>(is_too_close_and_distance);

        // A simple deceleration controler which breaks harder the closer an other vehicle gets
        other_car_speed = MetersPerSecondToMph(other_car_speed);
        auto our_speed = input.speed;
        auto speed_difference = other_car_speed - our_speed;

        auto target_acceleration = (kDistanceForFullBreak / (3.0 * distance_to_other_car)) * KDefaultAcceleration;
        target_acceleration -= ((0.8 * speed_difference) / kSpeedDifference) * KDefaultAcceleration;
        target_acceleration = std::min(target_acceleration, KDefaultAcceleration);

        target_speed_out -= target_acceleration;
    }
    else if (target_speed < KMaxSpeed)
    {
        target_speed_out += KDefaultAcceleration;
    }

    return std::make_pair(is_too_close_to_other_car, target_speed_out);
}

double PredictDistanceInGivenSeconds(double pred_seconds, double current_s, double speed, const OtherCar& other_car)
{
    auto predicted_other_car = other_car.frenet_location.s + pred_seconds * other_car.Speed2DMagnitude();
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
    auto kTimeToPredict = 3.0;

    std::sort(other_cars.begin(), other_cars.end(), smaller_frenet());

    std::cout << "Sorted cars s: ";
    std::for_each(
        other_cars.begin(), other_cars.end(), [](OtherCar car) { std::cout << car.frenet_location.s << " | "; });
    std::cout << std::endl;

    std::cout << "Current s: " << current_s << std::endl;

    auto behind_and_ahead = FindClosestCarsBehindAndAhead(current_s, other_cars);
    auto car_behind = behind_and_ahead.first;
    auto car_ahead = behind_and_ahead.second;

    bool ahead_ok = true;
    bool behind_ok = true;

    if (car_behind != other_cars.end())
    {
        std::cout << "car_behind s: " << car_behind->frenet_location.s << " | ";
        auto distance = abs(car_behind->frenet_location.s - current_s);
        std::cout << "dist_behind: " << distance << " | ";
        distance = PredictDistanceInGivenSeconds(kTimeToPredict, current_s, speed, *car_behind);
        std::cout << "predicted_dist_behind: " << distance << std::endl;
        behind_ok = (distance > kSafetyDistance);
    }
    if (car_ahead != other_cars.end())
    {
        std::cout << "car_ahead s: " << car_ahead->frenet_location.s << " | ";
        auto distance = abs(car_ahead->frenet_location.s - current_s);
        std::cout << "dist_ahead: " << distance << " | ";
        distance = PredictDistanceInGivenSeconds(kTimeToPredict, current_s, speed, *car_ahead);
        std::cout << "predicted_dist_ahead: " << distance << std::endl;
        ahead_ok = (distance > kSafetyDistance);
    }

    return ahead_ok && behind_ok;
}

bool IsSafeToChangeLaneOldApproach(double current_s, double speed, vector<OtherCar>& other_cars)
{
    auto kSafetyDistance = 15.0;
    std::sort(other_cars.begin(), other_cars.end(), smaller_frenet());

    std::cout << "Sorted cars s: ";
    std::for_each(
        other_cars.begin(), other_cars.end(), [](OtherCar car) { std::cout << car.frenet_location.s << " | "; });
    std::cout << std::endl;
    std::for_each(other_cars.begin(), other_cars.end(), [](OtherCar car) { std::cout << car.lane << " | "; });
    std::cout << std::endl;
    std::for_each(
        other_cars.begin(), other_cars.end(), [](OtherCar car) { std::cout << car.frenet_location.d << " | "; });
    std::cout << std::endl;

    std::cout << "Current s: " << current_s << std::endl;

    auto first_car_ahead = std::stable_partition(other_cars.begin(), other_cars.end(), [current_s](OtherCar car) {
        return (car.frenet_location.s > current_s);
    });

    auto closest_car_ahead = std::max_element(other_cars.begin(), first_car_ahead, smaller_frenet());
    auto closest_car_behind = std::max_element(first_car_ahead, other_cars.end(), smaller_frenet());

    bool ahead_ok = true;
    bool behind_ok = true;
    if (closest_car_behind != other_cars.end() && closest_car_behind >= other_cars.begin())
    {
        std::cout << "Closest car behind s: " << closest_car_behind->frenet_location.s << " | ";
        auto distance = abs(current_s - closest_car_behind->frenet_location.s);
        distance = PredictDistanceInGivenSeconds(1.0, current_s, speed, *closest_car_behind);
        std::cout << "dist_behind: " << distance << " | ";
        behind_ok = (distance > kSafetyDistance);
    }
    if (closest_car_ahead != other_cars.end() && closest_car_ahead >= other_cars.begin())
    {
        std::cout << "Closest car ahead s: " << closest_car_ahead->frenet_location.s << " | ";
        auto distance = abs(current_s - closest_car_ahead->frenet_location.s);
        distance = PredictDistanceInGivenSeconds(1.0, current_s, speed, *closest_car_ahead);
        std::cout << "dist_ahead: " << distance << " | ";
        ahead_ok = (distance > kSafetyDistance);
    }
    std::cout << std::endl;
    return ahead_ok && behind_ok;
}

bool IsLaneChangeFeasible(const PathPlannerInput& input, const int for_lane)
{
    auto lane_related_other_cars = GetLaneRelatedOtherCars(input.other_cars);
    auto is_safe = IsSafeToChangeLane(input.frenet_location.s, input.speed, lane_related_other_cars[for_lane]);
    return is_safe;
}

}  // namespace

void DrivingState::DefaultPrepareLaneChangeLogic(DataUpdate const& update, int final_lane)
{
    if (true)  // Alternatively something like "if (update.payload.speed > (KMaxSpeed - 15.0))"
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

// Motor states
class KeepingLane : public DrivingState
{
  private:
    void entry() override { std::cout << "Keeping Lane" << std::endl; }

    void react(DataUpdate const& update) override { DecideDrivingPolicyForSpeedAndLane(update.payload); }

    double GetSpeedOfCarAheadForLane(int lane, double current_s, const vector<OtherCar>& other_cars) const
    {
        auto speed = 0.0;
        const auto& lr_cars = GetLaneRelatedOtherCars(other_cars);
        if (lane <= 2 && lane >= 0)
        {
            const auto& lane_cars = lr_cars[lane];
            speed = GetSpeedOfCarAhead(current_s, lane_cars);
        }
        return speed;
    }

    double GetSpeedOfCarAhead(double current_s, const vector<OtherCar>& other_cars) const
    {
        double kDefaultValueIfNoCarSeen = 1000.0;
        double kRelevantDistanceConsidered = 150.0;
        double speed = kDefaultValueIfNoCarSeen;

        auto behind_and_ahead = FindClosestCarsBehindAndAhead(current_s, other_cars);
        const auto car_ahead = behind_and_ahead.second;
        if (car_ahead != other_cars.end())
        {
            auto distance = abs(car_ahead->frenet_location.s - current_s);
            if (distance < kRelevantDistanceConsidered)
            {
                speed = car_ahead->Speed2DMagnitude();
            }
        }

        return speed;
    }

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
            //            auto av_speed_current = GetAverageSpeed(target_lane_, input.other_cars);
            //            av_speed_current = 0.0;
            //            auto av_speed_left = GetAverageSpeed(target_lane_ - 1, input.other_cars);
            //            auto av_speed_right = GetAverageSpeed(target_lane_ + 1, input.other_cars);

            auto current_s = input.frenet_location.s;
            auto& other_cars = input.other_cars;
            auto av_speed_current = GetSpeedOfCarAheadForLane(target_lane_, current_s, other_cars);
            auto av_speed_left = GetSpeedOfCarAheadForLane(target_lane_ - 1, current_s, other_cars);
            auto av_speed_right = GetSpeedOfCarAheadForLane(target_lane_ + 1, current_s, other_cars);

            std::cout << "speed current: " << av_speed_current << " | av_speed_left: " << av_speed_left
                      << " | av_speed_right: " << av_speed_right << std::endl;

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
        else
        {
            std::cout << "Keeping lane at speed: " << target_speed_ << std::endl;
        }

        auto target_speed = lane_controller_output.second;
        target_speed_ = target_speed;
    }
};

class ChangingLaneRight : public DrivingState
{
    void entry() override
    {
        std::cout << "Changing lane right" << std::endl;
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
        std::cout << "Changing lane left" << std::endl;
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

class PreparingLaneChangeRight : public DrivingState
{
    void entry() override { std::cout << "Preparing to change lane right" << std::endl; }

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
    void entry() override { std::cout << "Preparing to change lane left" << std::endl; }

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
int DrivingState::target_lane_ = kCenter;
double DrivingState::target_speed_ = 0.0;

// Initial state definition
FSM_INITIAL_STATE(DrivingState, KeepingLane)
