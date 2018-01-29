#ifndef MOTOR_HPP_INCLUDED
#define MOTOR_HPP_INCLUDED

#include <PathPlannerInput.h>
#include <tinyfsm.hpp>

double Deg2Rad(double x);
double MphToMetersPerSecond(double mph_value);
double MetersPerSecondToMph(double mps_value);

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
    /* default reaction for unhandled events */
    void react(tinyfsm::Event const&){};

    virtual void react(DataUpdate const&);

    /* non-virtual declaration: reactions are the same for all states */
    void react(PrepareLaneChangeRightIntent const&);
    void react(PrepareLaneChangeLeftIntent const&);
    void react(ChangeLaneRightIntent const&);
    void react(ChangeLaneLeftIntent const&);
    void react(LaneChangeCompleted const&);
    void react(AbortLaneChange const&);

    virtual void entry(void) = 0; /* pure virtual: enforce implementation in all states */
    void exit(void){};            /* no exit actions at all */

    double GetTargetSpeed() const { return target_speed_; }
    double GetTargetLane() const { return target_lane_; }

  protected:
    void DefaultPrepareLaneChangeLogic(DataUpdate const& update, int lane, tinyfsm::Event event);

    static PathPlannerInput input_;
    static int target_lane_;
    static double target_speed_;
};

#endif
