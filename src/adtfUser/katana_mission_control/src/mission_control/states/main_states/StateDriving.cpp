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

#include "mission_control/states/main_states/StateDriving.h"

namespace katana
{

StateDriving::StateDriving(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
  : MainStateBase(system, driving_strip_changer)
{
  createDrivingStateMachine(system, m_driving_strip_changer);
}

void StateDriving::createDrivingStateMachine(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
{
  // Create Driving states
  std::array<DriveBase::Ptr, (u_int32_t)DrivingState::DRIVE_STATE_COUNT> drive_states;
  drive_states[(u_int32_t)DrivingState::DRIVE_NORMAL].reset(new DriveNormal(m_system, driving_strip_changer));
  drive_states[(u_int32_t)DrivingState::DRIVE_INITIALIZE].reset(new DriveInitialize(m_system, driving_strip_changer));
  drive_states[(u_int32_t)DrivingState::DRIVE_PARKING].reset(new DriveParking(m_system, driving_strip_changer));
  drive_states[(u_int32_t)DrivingState::DRIVE_JUNCTION].reset(new DriveJunction(m_system, driving_strip_changer));
  drive_states[(u_int32_t)DrivingState::DRIVE_PULLOUT].reset(new DrivePullOut(m_system, driving_strip_changer));
  drive_states[(u_int32_t)DrivingState::DRIVE_FINISHED].reset(new DriveFinished(m_system, driving_strip_changer));
  m_driving_state.reset(new StateManager<DriveBase, DrivingState, DrivingState::DRIVE_STATE_COUNT>(drive_states));
}

void StateDriving::poseChanged(const PoseWithTime& pose_stamped)
{
  // let driving state do something (e.g. update planned motion)
  m_driving_state->getStateObject()->poseChanged(pose_stamped);
}

bool StateDriving::juryRequestState()
{
  return m_driving_state->getStateObject()->juryRequestState();
}

} // ns
