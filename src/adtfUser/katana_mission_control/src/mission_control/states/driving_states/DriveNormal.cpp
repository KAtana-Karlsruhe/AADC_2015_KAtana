// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-12
 *
 */
//----------------------------------------------------------------------

#include "mission_control/states/driving_states/DriveNormal.h"

namespace katana
{

void DriveNormal::onActivation(u_int32_t previous)
{
#ifdef KATANA_MC_STATE_DEBUG
  std::cout <<"MissionControl - DrivingState: NORMAL" <<std::endl;
#endif

  // Speed up AFTER a certain distance
  m_waiting_for_speed_up = true;
  m_activation_distance = m_system->getPositionController()->getOverallDrivenDistance();
  if (previous == (u_int32_t)DrivingState::DRIVE_JUNCTION)
  {
    m_distance_to_wait = m_system->getConfig()->getDouble(CONF_DISTANCE_NORMAL_SPEED_AFTER_JUNCTION);
  }
  else
  {
    m_distance_to_wait = m_system->getConfig()->getDouble(CONF_DISTANCE_NORMAL_SPEED);
  }

  m_system->resetBlockedArduinoCounter();
  m_system->resetBlockedByObstacleCounter();

  m_system->updatePlannedMotion();

  // is lanetracker is not working, turn it on now
  if (!m_system->getLanetrackerJobManager()->isLTWorking())
    m_system->getLanetrackerJobManager()->submitJob(m_system->getWorld()->getNextPatches(), NORMAL, 0, 0, -1.0);

  // Turn on lights
  m_system->getLightController()->setLight(LightName::HEAD_LIGHT, true);
}

void DriveNormal::poseChanged(const PoseWithTime& pose)
{
  // do nothing... (timerUpdate checks for driven distance and point generation
}

void DriveNormal::timerUpdate()
{
  // check if vehicle is physically blocked, e.g. after arduino crash
  m_system->timerCheckIfBlocked();

  if (m_waiting_for_speed_up)
  {
    if (m_system->getPositionController()->getOverallDrivenDistance() - m_distance_to_wait > m_activation_distance)
    {
      m_waiting_for_speed_up = false;
      m_system->getMotionPlanning()->defaultSpeed() = m_system->getConfig()->getDouble(DoubleParameter::CONF_STANDARD_VELOCITY);
    }
  }

  if (m_driving_strip_changer->isCurrentlyLaneChanging() || m_driving_strip_changer->isCurrentylReversing())
  {
    m_system->getMotionPlanning()->defaultSpeed() = m_system->getConfig()->getDouble(DoubleParameter::CONF_VELOCITY_AFTER_LANE_CHANGE);
    if (m_system->getPositionController()->mustNotMove(StopReason::OBSTACLE))
      m_system->getPositionController()->go(StopReason::OBSTACLE);
  }

  // check if vehicle is blocked by obstacle
  else if (m_system->timerCheckIfObstacleOnTrajectory())
  {
    if (m_system->getBlockedByObstacleCounter() > (u_int32_t)m_system->getConfig()->getDouble(CONF_OBSTACLE_WAITING_DURATION) && !m_driving_strip_changer->isCurrentylReversing())
    {
      m_system->resetBlockedByObstacleCounter();


      const double dist = m_driving_strip_changer->needToReverse();

      if (dist >= 0.0)
      {
        m_driving_strip_changer->changeLane(true);
      }
      else
      {
        m_driving_strip_changer->driveReverse(-dist, true);
      }
    }
  }

  // Check for driven distance and update planning
  // if hasReached() is true, also try to generate a new driving strip
  if (!m_driving_strip_changer->isCurrentylReversing())   //< only when not controlled by driving strip changer
    m_system->timerUpdateTrajectory();

  // We are at end of driving strip and no new patches are known. Switch to DRIVE INITIAL state
  if(m_system->getPositionController()->hasReached()) {
    m_change_state((u_int32_t)DrivingState::DRIVE_INITIALIZE);
  }
}

void DriveNormal::newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id)
{
  // lanetracker didn't see anything, we are at end of trajectory -> change to initialise
  if (num == 0 && m_system->getPositionController()->hasReached() && !m_driving_strip_changer->isCurrentylReversing())
  {
    m_change_state((u_int32_t)DrivingState::DRIVE_INITIALIZE);
  }
  else
  {
    m_system->getLanetrackerJobManager()->submitJob(m_system->getWorld()->getNextPatches(), NORMAL, 0, 0, -1.0);
  }

}

void DriveNormal::newTrafficSign(RoadSign::Ptr sign, bool signAppeared)
{
  katana::DriveBase::newTrafficSign(sign, signAppeared);

  DrivingState nextState = m_system->getNextState(sign);
  if(nextState != DrivingState::DRIVE_STATE_COUNT) {
    m_change_state((u_int32_t)nextState);
  }
}

} // ns
