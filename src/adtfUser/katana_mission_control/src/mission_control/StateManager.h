// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-09
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_STATEMANAGER_H
#define _MISSION_CONTROL_STATEMANAGER_H

#include "katanaCommon/katanaCommon.h"
#ifdef KATANA_MC_STATE_DEBUG
#include <iostream>
#endif

#include "PoseStamped.h"

/** ******** STATES **********/
#include "mission_control/StateBase.h"

namespace katana
{

/**
 * @brief The StateManager class
 */
template<class T, class N, N _count>
class StateManager
{
public:
  //! Convenience Ptr
  typedef std::shared_ptr<StateManager> Ptr;
  typedef std::shared_ptr<T> StateTypePtr;

  typedef std::array<StateTypePtr, (u_int32_t)_count> StateCollection;

  //! Constructor
  StateManager() = delete;
  StateManager(const std::array<StateTypePtr, (u_int32_t)_count>& states)
    : m_leaving_state(false)
  {
    // save states
    m_state_container = states;
    // init states
    for (StateTypePtr& i : m_state_container)
    {
      i->init(std::bind(&StateManager::changeStateInternal, this, std::placeholders::_1));
    }
  }

  void changeState(N new_state)       { changeStateInternal((u_int32_t)new_state); }

  //! Get current state
  N getState() const
  {
    if (!m_state)
      return (N)0;   // <do we need an error code?>
    return (N)(m_state->getState());
  }

  //! Return the state object
  const StateTypePtr& getStateObject()     { return m_state; }

  //! Tell current state that this state machine will be suspended
  void suspend()                           { if (m_state != NULL) { m_state->onStateLeave(m_state->getState()); } }
  //! Tell current state to reactivate
  void reactivate()                        { if (m_state != NULL) { m_state->onActivation(m_state->getState()); } }

  bool hasState() const   { return m_state != NULL; }

  //! Access to all states
  const StateCollection& getStateContainer() const   { return m_state_container; }

private:
  //! Set current state
  void changeStateInternal(u_int32_t new_state)
  {
    assert(new_state < m_state_container.size());

    u_int32_t old_state = 0;

    if (m_state != NULL)                        //< check for previous state
    {
      old_state = m_state->getState();

      if (old_state == new_state)               //< same state
        return;

      if (!m_leaving_state)                     //< do not recursivly call onleave
      {
        m_leaving_state = true;                 //< remember this
        const bool can_leave = m_state->onStateLeave(new_state);
        m_leaving_state = false;

        if (!can_leave)
          return;
      }
    }
    m_state = m_state_container[new_state];
    m_state->onActivation(old_state);
  }

  //! Current state
  StateTypePtr m_state;

  //! Flag: if true, current state is currently in state leave
  bool m_leaving_state;

  //! All state instances
  StateCollection m_state_container;
};

} // ns

#endif //_MISSION_CONTROL_STATEMANAGER_H
