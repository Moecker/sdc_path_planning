#include "DrivingStateMachine.h"
#include <iostream>
#include <tinyfsm.hpp>

using namespace std;

// Motor states
class KeepingLane : public DrivingState
{
    void entry() override { cout << "Keeping Lane" << endl; };
};

class ChangingLaneRight : public DrivingState
{
    void entry() override { cout << "Changing lane right" << endl; };
};

class ChangingLaneLeft : public DrivingState
{
    void entry() override { cout << "Changing lane left" << endl; };
};

class PreparingLaneChangeRight : public DrivingState
{
    void entry() override { cout << "Preparing to change lane right" << endl; };
};

class PreparingLaneChangeLeft : public DrivingState
{
    void entry() override { cout << "Preparing to change lane left" << endl; };
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
    UpdateWithCurrentInput(update.payload);
}

// Initial state definition
FSM_INITIAL_STATE(DrivingState, KeepingLane)
