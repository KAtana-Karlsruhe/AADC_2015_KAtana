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

#ifndef _MISSION_CONTROL_DRIVEBASE_H
#define _MISSION_CONTROL_DRIVEBASE_H

#include "katanaCommon/katanaCommon.h"
#include "PoseStamped.h"
#include "mission_control/states/main_states/MainStateBase.h"
#include "mission_control/System.h"


namespace katana
{

class DriveBase : public StateBase
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<DriveBase> Ptr;

  //! Constructor
  DriveBase() = delete;
  DriveBase(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : StateBase()
    , m_system(system)
    , m_driving_strip_changer(driving_strip_changer)
  {

  }

  //! Destructor
  virtual ~DriveBase()  {}

  //! Jury requests the state. Return true if ready, false if not
  virtual bool juryRequestState() 			{ return false; }

  //! New pose
  virtual void poseChanged(const PoseWithTime& pose)      {}

  //! New patch
  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) {}

  //! Lanetracker has answered
  virtual bool lanetrackerCallback(u_int32_t job_id)      { return true; }

  //! New Obstacle
  virtual void newObstacle()			       {}

  //! Timer
  virtual void timerUpdate()                             {}

  //! New Traffic Sign
  virtual void newTrafficSign(RoadSign::Ptr sign, bool signAppeared = true)	       {}

protected:
  //! System
  System::Ptr m_system;

  //! Driving strip
  DrivingStripChanger::Ptr m_driving_strip_changer;
};


} // ns

#endif //_MISSION_CONTROL_DRIVEBASE_H
