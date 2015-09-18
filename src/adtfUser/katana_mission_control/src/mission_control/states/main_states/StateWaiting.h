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

#ifndef _MISSION_CONTROL_STATEWAITING_H
#define _MISSION_CONTROL_STATEWAITING_H

#include "katanaCommon/katanaCommon.h"
#include "mission_control/states/main_states/MainStateBase.h"


namespace katana
{

class StateWaiting : public MainStateBase
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<StateWaiting> Ptr;

  //! Constructor
  StateWaiting(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : MainStateBase(system, driving_strip_changer)
  {}

  //! Destructor
  virtual ~StateWaiting()      {}

  virtual void juryRunCommand(int16_t maneuver_entry)
  {
    if (!m_system->getLanetrackerJobManager()->isLTReady())
    {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout <<"Can not change to driving, lanetracker is not ready yet!" <<std::endl;
#endif
      return;
    }

    m_change_state((u_int32_t)MainState::DRIVING);
  }

  //! Get current state
  virtual u_int32_t getState() const override    { return (u_int32_t)MainState::WAITING; }

  //!
  virtual void onActivation(u_int32_t previous) override;

  //! Gets called regularly
  virtual void timerUpdate() override
  {
    if (!m_system->getLanetrackerJobManager()->isLTReady())
      m_system->getLanetrackerJobManager()->pingLanetracker();
  }

private:
};


} // ns

#endif //_MISSION_CONTROL_STATEWAITING_H
