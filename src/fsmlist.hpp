#ifndef FSMLIST_HPP_INCLUDED
#define FSMLIST_HPP_INCLUDED

#include <tinyfsm.hpp>
#include "DrivingStateMachine.h"

typedef tinyfsm::FsmList<Motor> FsmList;

template <typename E>
void SendEvent(E const& event)
{
    FsmList::template dispatch<E>(event);
}

#endif
