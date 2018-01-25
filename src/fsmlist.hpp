#ifndef FSMLIST_HPP_INCLUDED
#define FSMLIST_HPP_INCLUDED

#include <tinyfsm.hpp>

#include "DrivingStateMachine.h"

typedef tinyfsm::FsmList<Motor> FsmList;

/* wrapper to fsm_list::dispatch() */
template<typename E>
void send_event(E const & event)
{
  FsmList::template dispatch<E>(event);
}


#endif
