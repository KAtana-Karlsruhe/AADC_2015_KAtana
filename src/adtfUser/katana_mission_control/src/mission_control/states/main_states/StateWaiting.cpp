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

#include "mission_control/states/main_states/StateWaiting.h"

namespace katana
{


void StateWaiting::onActivation(u_int32_t previous)
{
  //stop vehicle
  m_system->getPositionController()->stop(StopReason::JURY);

#ifdef KATANA_MC_STATE_DEBUG
  std::cout <<"MissionControl - MainState: WAITING" <<std::endl;
#endif

  // ping lanetracker if not already ready
  if (!m_system->getLanetrackerJobManager()->isLTReady())
    m_system->getLanetrackerJobManager()->pingLanetracker();

  m_system->getMotionPlanning()->defaultSpeed() = 0.25;
}

} // ns
