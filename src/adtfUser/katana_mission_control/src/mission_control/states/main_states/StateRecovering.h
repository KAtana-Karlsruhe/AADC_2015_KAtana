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

#ifndef _MISSION_CONTROL_STATERECOVERING_H
#define _MISSION_CONTROL_STATERECOVERING_H

#include "katanaCommon/katanaCommon.h"
#include "mission_control/states/main_states/MainStateBase.h"
#include "Pose.h"

namespace katana
{

class StateRecovering : public MainStateBase
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<StateRecovering> Ptr;

  //! Constructor
  StateRecovering(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : MainStateBase(system, driving_strip_changer)
  {}

  //! Destructor
  virtual ~StateRecovering()      {}

  //! Get current state
  virtual u_int32_t getState() const override   { return (u_int32_t)MainState::RECOVERING; }

  //!
  virtual void juryRunCommand(int16_t maneuver_entry) override;
  
  //!
  virtual void onActivation(u_int32_t previous) override;

private:

};

} // ns

#endif //_MISSION_CONTROL_STATERECOVERING_H
