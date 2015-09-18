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

#ifndef _MISSION_CONTROL_STATEDRIVING_H
#define _MISSION_CONTROL_STATEDRIVING_H

#include "katanaCommon/katanaCommon.h"
#include "mission_control/states/main_states/MainStateBase.h"
#include "mission_control/StateManager.h"

#include "mission_control/states/driving_states/DriveNormal.h"
#include "mission_control/states/driving_states/DriveInitialize.h"
#include "mission_control/states/driving_states/DriveParking.h"
#include "mission_control/states/driving_states/DriveJunction.h"
#include "mission_control/states/driving_states/DrivePullOut.h"
#include "mission_control/states/driving_states/DriveFinished.h"

#include "Pose.h"


namespace katana
{

class StateDriving : public MainStateBase
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<StateDriving> Ptr;

  //! Constructor
  StateDriving(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer);

  //! Destructor
  virtual ~StateDriving()      {}

  //! Jury requests the state. Return true if ready, false if not
  virtual bool juryRequestState() override;

  //! Lanetracker has answered
  virtual bool lanetrackerCallback(u_int32_t job_id) override    { return m_driving_state->getStateObject()->lanetrackerCallback(job_id); }

  //! Get current state
  virtual u_int32_t getState() const override    { return (u_int32_t)MainState::DRIVING; }

  //!
  virtual void poseChanged(const PoseWithTime& pose_stamped) override;

  //! Control underlying driving state machine and set VelocityControl
  virtual void onActivation(u_int32_t previous_state) override
  {
#ifdef KATANA_MC_STATE_DEBUG
  std::cout <<"MissionControl - MainState: DRIVING" <<std::endl;
#endif

    if (m_driving_state->hasState())
    {
      m_driving_state->reactivate();
    }
    else
    {
      // first activation of driving state: change to initialize state
      m_driving_state->changeState(DrivingState::DRIVE_INITIALIZE);
    }

    // go...
    m_system->getPositionController()->go(StopReason::JURY);
  }

  virtual bool onStateLeave(u_int32_t next_state) override
  {
    // suspend driving state machine
    m_driving_state->suspend();
    // allow state changing
    return true;
  }

  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) override
  {
    // notify driving state
    if (m_driving_state->hasState())
      m_driving_state->getStateObject()->newPatch(num, perception_state, job_id);
  }

  //! Gets called regularly
  virtual void timerUpdate() override
  {
    m_driving_state->getStateObject()->timerUpdate();
  }

  //! New Traffic Sign
  virtual void newTrafficSign(RoadSign::Ptr sign, bool signAppeared = true) override
  {
    m_driving_state->getStateObject()->newTrafficSign(sign, signAppeared);
  }

  //! Return driving state machine
  DriveBase::Ptr getDrivingState()      { return m_driving_state->getStateObject(); }

  //! Reset driving state
  void resetStateMachine()
  {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
    std::cout <<"[StateDriving] Resetting driving state machine" <<std::endl;
#endif

    // re-create driving state machine
    createDrivingStateMachine(m_system, m_driving_strip_changer);

  }

private:
  //! Driving state machine
  StateManager<DriveBase, DrivingState, DrivingState::DRIVE_STATE_COUNT>::Ptr m_driving_state;

  void createDrivingStateMachine(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer);
};

} // ns

#endif //_MISSION_CONTROL_STATEDRIVING_H
