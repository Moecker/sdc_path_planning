#include "DrivingStateMachine.h"
#include "SimpleSplineBasedPlanner.h"

#include <fsmlist.hpp>
#include <iostream>
#include <tinyfsm.hpp>
#include <typeinfo>

using namespace std;

// Motor states
class KeepingLane : public DrivingState
{
    double MphToMetersPerSecond(double mph_value) { return mph_value / 2.24; }
    double MetersPerSecondToMph(double mps_value) { return mps_value * 2.24; }

    const double KDefaultAcceleration = 0.447;
    const double KMaxSpeed = 69.5;
    const double kCriticalThresholdInMeters = 25.0;
    const double kSimulatorRunloopPeriod = 0.02;
    const double kXAxisPlanningHorizon = 50.0;

    static const int kMaxNumberOfPointsInPath = 50;
    static const int kLeftmostLaneNumber = 0;

    void entry() override { cout << "Keeping Lane" << endl; };

    void react(DataUpdate const& update) override { DecideDrivingPolicyForSpeedAndLane(update.payload); };

    void DecideDrivingPolicyForSpeedAndLane(const PathPlannerInput& input)
    {
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

            target_speed_ -= target_acceleration;

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
        else if (target_speed_ < KMaxSpeed)
        {
            target_speed_ += KDefaultAcceleration;
        }
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
                auto predicted_increase_of_s_wrt_our_car =
                    kSimulatorRunloopPeriod * other_car_speed;

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
    };
};

class PreparingLaneChangeRight : public DrivingState
{
    void entry() override { cout << "Preparing to change lane right" << endl; };

    void react(DataUpdate const& update) override
    {
        if (update.payload.lane < 2)
        {
            SendEvent(ChangeLaneRightIntent());
        }
        else
        {
            SendEvent(AbortLaneChange());
        }
    };
};

class PreparingLaneChangeLeft : public DrivingState
{
    void entry() override { cout << "Preparing to change lane left" << endl; };

    void react(DataUpdate const& update) override
    {
        if (update.payload.lane > 0)
        {
            SendEvent(ChangeLaneLeftIntent());
        }
        else
        {
            SendEvent(AbortLaneChange());
        }
    };
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
