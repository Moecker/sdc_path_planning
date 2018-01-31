///
/// @file
///

#ifndef FSMLIST_HPP_INCLUDED
#define FSMLIST_HPP_INCLUDED

#include <tinyfsm.hpp>
#include "DrivingStateMachine.h"

typedef tinyfsm::FsmList<DrivingState> FsmList;

// This method does the FSM magic by referencing the state action (entry, exit, ...) to the current state.
template <typename E>
void SendEvent(E const& event)
{
    FsmList::template dispatch<E>(event);
}

#endif
