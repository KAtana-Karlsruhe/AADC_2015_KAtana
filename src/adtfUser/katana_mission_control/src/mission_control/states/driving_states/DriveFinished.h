// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-16
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_DRIVEFINISHED_H
#define _MISSION_CONTROL_DRIVEFINISHED_H

#include "katanaCommon/katanaCommon.h"
#include "Pose.h"
#include "mission_control/states/driving_states/DriveBase.h"

namespace katana {

class DriveFinished :  public DriveBase
{
public:

  //! Constructor
  DriveFinished(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : DriveBase(system, driving_strip_changer)
  {

  }

  //!
  u_int32_t getState() const override   { return (u_int32_t)DrivingState::DRIVE_FINISHED; }

  //!
  virtual void onActivation(u_int32_t previous) override {
    #ifdef KATANA_MC_STATE_DEBUG
      std::cout <<"MissionControl - DrivingState: FINNISHED" <<std::endl;
    #endif

    m_system->callDriverState()(SendAction::COMPLETE,(int16_t)m_system->getManeuver()->getCurrentManeuverId());
    m_system->getPositionController()->stop(StopReason::MANEUVER_COMPLETE);

    m_system->getLightController()->setLight(LightName::WARNING_LIGHTS, true);
  }

  //!
  virtual void timerUpdate() override {}

  //!
  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) override {}

private:



};

}
#endif // _MISSION_CONTROL_DRIVEFINISHED_H
