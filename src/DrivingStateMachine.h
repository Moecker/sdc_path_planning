#ifndef MOTOR_HPP_INCLUDED
#define MOTOR_HPP_INCLUDED

#include <PathPlannerInput.h>
#include <tinyfsm.hpp>

// Event declarations
struct PrepareLaneChangeRightIntent : tinyfsm::Event
{
};

struct PrepareLaneChangeLeftIntent : tinyfsm::Event
{
};

struct ChangeLaneRightIntent : tinyfsm::Event
{
};

struct ChangeLaneLeftIntent : tinyfsm::Event
{
};

struct LaneChangeCompleted : tinyfsm::Event
{
};

struct AbortLaneChange : tinyfsm::Event
{
};

struct DataUpdate : tinyfsm::Event
{
    DataUpdate() = default;
    DataUpdate(PathPlannerInput payload0) : payload(payload0) {}

    PathPlannerInput payload;
};

// Motor (FSM base class) declaration
class DrivingState : public tinyfsm::Fsm<DrivingState>
{
    /* NOTE: react(), entry() and exit() functions need to be accessible
     * from tinyfsm::Fsm class. You might as well declare friendship to
     * tinyfsm::Fsm, and make these functions private:
     *
     * friend class Fsm;
     */
  public:
    DrivingState() = default;

    void UpdateWithCurrentInput(const PathPlannerInput& input) { input_ = input; };

    /* default reaction for unhandled events */
    void react(tinyfsm::Event const&){};
    void react(DataUpdate const&);

    /* non-virtual declaration: reactions are the same for all states */
    void react(PrepareLaneChangeRightIntent const&);
    void react(PrepareLaneChangeLeftIntent const&);
    void react(ChangeLaneRightIntent const&);
    void react(ChangeLaneLeftIntent const&);
    void react(LaneChangeCompleted const&);
    void react(AbortLaneChange const&);

    virtual void entry(void) = 0; /* pure virtual: enforce implementation in all states */
    void exit(void){};            /* no exit actions at all */

  private:
    PathPlannerInput input_;
};

#endif
