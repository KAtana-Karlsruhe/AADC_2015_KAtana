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

#include "mission_control/states/main_states/StateRecovering.h"

namespace katana
{

void StateRecovering::juryRunCommand(int16_t maneuver_entry)
{
  // do nothing -> override standard behaviour to switch into driving
}

void StateRecovering::onActivation(u_int32_t previous)
{
  katana::StateBase::onActivation(previous);
  #ifdef KATANA_MC_STATE_DEBUG
    std::cout <<"MissionControl - MainState: RECOVERING" <<std::endl;
  #endif
}

} // ns
