// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-08
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_STATE_BASE_H
#define _MISSION_CONTROL_STATE_BASE_H

#include "katanaCommon/katanaCommon.h"
#include "Pose.h"

#ifdef KATANA_MC_STATE_DEBUG
#include <iostream>
#endif

namespace katana
{

class StateBase
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<StateBase> Ptr;

  //! Constructor
  StateBase()               {}

  //! Destructor
  virtual ~StateBase()      {}

  //! Initialization by state manager
  void init(const std::function<void(u_int32_t)>& change_state);

  //! Get current state
  virtual u_int32_t getState() const = 0;

  //! Called by state manager after state has been activated
  virtual void onActivation(u_int32_t previous_state)   {}

  //! Called by state manager when state will be left, possibility to cancel
  virtual bool onStateLeave(u_int32_t next_state)   { return true; }

  virtual void resetStateMachine()	{}

protected:
  //!
  std::function<void(u_int32_t)> m_change_state;
};


} // ns

#endif //_MISSION_CONTROL_STATE_BASE_H
