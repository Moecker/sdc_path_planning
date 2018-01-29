#include "DrivingStateMachine.h"
#include "SimpleSplineBasedPlanner.h"

#include <fsmlist.hpp>
#include <iostream>
#include <tinyfsm.hpp>
#include <typeinfo>

using namespace std;

static const double KDefaultAcceleration = 0.447;
static const double KMaxSpeed = 49.5;
static const double kCriticalThresholdInMeters = 25.0;
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
        if (lane < 2 && lane > 0)
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
        const auto kDistanceForFullBreak = 10.0;
        const auto kSpeedDifference = 10.0;

        auto is_too_close_and_distance = IsTooCloseToOtherCar(input);

        auto is_too_close_to_other_car = std::get<0>(is_too_close_and_distance);
        auto distance_to_other_car = std::get<1>(is_too_close_and_distance);
        auto other_car_speed = std::get<2>(is_too_close_and_distance);

        // DebugScenario();

        if (is_too_close_to_other_car)
        {
            // A simple deceleration controler which breaks harder the closer an other vehicle gets
            other_car_speed = MetersPerSecondToMph(other_car_speed);
            auto our_speed = input.speed;
            auto speed_difference = other_car_speed - our_speed;

            auto target_acceleration = (kDistanceForFullBreak / (2 * distance_to_other_car)) * KDefaultAcceleration;
            target_acceleration -= ((0.5 * speed_difference) / kSpeedDifference) * KDefaultAcceleration;
            target_acceleration = std::min(target_acceleration, KDefaultAcceleration);

            target_speed_ -= target_acceleration;

            auto av_speed_left = GetAverageSpeed(target_lane_ - 1, input.other_cars);
            auto av_speed_right = GetAverageSpeed(target_lane_ + 1, input.other_cars);

            if (av_speed_left > av_speed_right)
            {
                SendEvent(PrepareLaneChangeLeftIntent());
            }
            else
            {
                SendEvent(PrepareLaneChangeRightIntent());
            }
        }
        else if (target_speed_ < KMaxSpeed)
        {
            target_speed_ += KDefaultAcceleration;
        }
    }

    void DebugScenario()
    {
        target_speed_ = 49.5;
        srand(time(NULL));
        auto left = PrepareLaneChangeLeftIntent();
        auto right = PrepareLaneChangeRightIntent();

        int choice[2] = {0, 1};
        int decision = choice[rand() % 2];

        if (decision == 1)
            SendEvent(left);
        else
            SendEvent(right);
    }

    std::tuple<bool, double, double> IsTooCloseToOtherCar(const PathPlannerInput& input) const
    {
        double ego_predicted_end_point_s =
            !input.previous_path.empty() ? input.path_endpoint_frenet.s : input.frenet_location.s;

        for (auto& other_car : input.other_cars)
        {
            if (other_car.IsInLane(target_lane_))
            {
                auto other_car_speed = other_car.Speed2DMagnitude();
                auto predicted_increase_of_s_wrt_our_car = kSimulatorRunloopPeriod * other_car_speed;

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

bool IsSafeToChangeLane(double current_s, double speed, vector<OtherCar>& other_cars)
{
    auto closest_car_behind = std::find_if(other_cars.begin(), other_cars.end(), [current_s](OtherCar car) {
        return (car.frenet_location.s < current_s);
    });

    auto closest_car_ahead = std::find_if(other_cars.begin(), other_cars.end(), [current_s](OtherCar car) {
        return (car.frenet_location.s > current_s);
    });

    bool ahead_ok, behind_ok = true;
    if (closest_car_behind != other_cars.end())
    {
        auto distance = abs(current_s - closest_car_behind->frenet_location.s);
        distance = PredictDistanceInGivenSeconds(3.0, current_s, speed, *closest_car_behind);
        ahead_ok = (distance > 50.0);
    }
    if (closest_car_ahead != other_cars.end())
    {
        auto distance = abs(current_s - closest_car_ahead->frenet_location.s);
        behind_ok = (distance > 50.0);
    }

    return ahead_ok && behind_ok;
}

bool IsLaneChangeFeasible(const PathPlannerInput& input, const int for_lane)
{
    auto lane_related_other_cars = GetLaneRelatedOtherCars(input.other_cars);
    auto is_safe = IsSafeToChangeLane(input.frenet_location.s, input.speed, lane_related_other_cars[for_lane]);
    return is_safe;
}

void DrivingState::DefaultPrepareLaneChangeLogic(DataUpdate const& update, int lane, tinyfsm::Event event)
{
    if ((update.payload.lane < 2 && update.payload.lane > 0) || update.payload.speed < (KMaxSpeed - 10.0))
    {
        auto is_feasible = IsLaneChangeFeasible(update.payload, target_lane_ + 1);
        if (is_feasible)
        {
            SendEvent(event);
        }
        else
        {
            target_speed_ -= KDefaultAcceleration / 2.0;
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

    void react(DataUpdate const& update) override { DefaultPrepareLaneChangeLogic(update, 2, ChangeLaneRightIntent()); }
};

class PreparingLaneChangeLeft : public DrivingState
{
    void entry() override { cout << "Preparing to change lane left" << endl; }

    void react(DataUpdate const& update) override { DefaultPrepareLaneChangeLogic(update, 0, ChangeLaneLeftIntent()); }
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
