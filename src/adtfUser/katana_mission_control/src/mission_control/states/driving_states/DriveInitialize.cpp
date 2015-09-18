// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-12
 *
 */
//----------------------------------------------------------------------

#include "mission_control/states/driving_states/DriveInitialize.h"

namespace katana
{

void DriveInitialize::onActivation(u_int32_t previous)
{
#ifdef KATANA_MC_STATE_DEBUG
  std::cout <<"MissionControl - DrivingState: INITIALIZE" <<std::endl;
#endif

  // the only case the car has to do something other than initialize and drive normal,
  // is when the car needs to start from a parking spot
  const Action command = m_system->getManeuver()->getCurrentManeuver();
  if (command == Action::PULL_OUT_LEFT || command == Action::PULL_OUT_RIGHT)
  {
    m_change_state((u_int32_t)DrivingState::DRIVE_PULLOUT);
    return;
  }

  // only submit job if lanetracker is not working
  if (!m_system->getLanetrackerJobManager()->isLTWorking())
    m_system->callLanetrackerInitial(0, oadrive::vision::PatchesToLookFor::DEFAULT, -1.0);
}

void DriveInitialize::poseChanged(const PoseWithTime& pose)
{
  // do nothing... (timerUpdate checks for driven distance and point generation
}

void DriveInitialize::timerUpdate()
{
  m_system->timerCheckIfObstacleOnTrajectory();
}

void DriveInitialize::newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id)
{
  if (num > 0)
  {
    // let lanetracker continue search
    // transmit world patches which seem to be "the street" to lanetracker
    m_system->getLanetrackerJobManager()->submitJob(m_system->getWorld()->getNextPatches(), NORMAL, 0, 0, -1.0);

    // we have obtained a patch! change to driving state an let driving state drive!
    m_change_state((u_int32_t)DrivingState::DRIVE_NORMAL);
  }
  else
  {
    m_system->callLanetrackerInitial(0, oadrive::vision::PatchesToLookFor::DEFAULT, -1.0);
  }
}

void DriveInitialize::newTrafficSign(RoadSign::Ptr sign, bool signAppeared)
{
  katana::DriveBase::newTrafficSign(sign, signAppeared);

  DrivingState nextState = m_system->getNextState(sign);
  if(nextState != DrivingState::DRIVE_STATE_COUNT) {
    m_change_state((u_int32_t)nextState);
  }
}


} // ns
