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

#ifndef _MISSION_CONTROL_DRIVENORMAL_H
#define _MISSION_CONTROL_DRIVENORMAL_H

#include "katanaCommon/katanaCommon.h"
#include "mission_control/states/driving_states/DriveBase.h"

namespace katana
{

class DriveNormal : public DriveBase
{
public:

  //! Constructor
  DriveNormal(const System::Ptr& system, const DrivingStripChanger::Ptr& driving_strip_changer)
    : DriveBase(system, driving_strip_changer)
  {

  }

  //!
  u_int32_t getState() const override   { return (u_int32_t)DrivingState::DRIVE_NORMAL; }

  //! Jury requests the state. Return true if ready, false if not
  virtual bool juryRequestState() override { return true; }

  //! Pose has changed
  virtual void poseChanged(const PoseWithTime& pose) override;

  //! State activated
  virtual void onActivation(u_int32_t previous) override;

  //! New patch in world
  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) override;

  //! New traffic sign
  virtual void newTrafficSign(RoadSign::Ptr sign, bool signAppeared = true) override;

  //! Timer
  virtual void timerUpdate() override;


private:

  //! Flag if waiting for speed up
  bool m_waiting_for_speed_up;
  double m_activation_distance;
  double m_distance_to_wait;

};


} // ns

#endif //_MISSION_CONTROL_DRIVENORMAL_H
