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

#ifndef _MISSION_CONTROL_DRIVEINITIALIZE_H
#define _MISSION_CONTROL_DRIVEINITIALIZE_H

#include "katanaCommon/katanaCommon.h"
#include "mission_control/states/driving_states/DriveBase.h"

namespace katana
{

/**
 * @brief The DriveInitialize class
 * This state assumes that the car stands on a patch
 */
class DriveInitialize : public DriveBase
{
public:

  //! Constructor
  DriveInitialize(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : DriveBase(system, driving_strip_changer)
  {

  }

  //!
  u_int32_t getState() const override   { return (u_int32_t)DrivingState::DRIVE_INITIALIZE; }

  //! Pose has changed
  virtual void poseChanged(const PoseWithTime& pose) override;

  //! State activated
  virtual void onActivation(u_int32_t previous) override;

  //! New patch in world
  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) override;

  //! Timer
  virtual void timerUpdate() override;

  //! New traffic sign
  virtual void newTrafficSign(RoadSign::Ptr sign, bool signAppeared = true) override;

private:

};


} // ns

#endif //_MISSION_CONTROL_DRIVEINITIALIZE_H
