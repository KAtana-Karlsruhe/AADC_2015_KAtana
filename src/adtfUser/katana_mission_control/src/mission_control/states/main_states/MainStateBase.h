// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-15
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_MAINSTATE_BASE_H
#define _MISSION_CONTROL_MAINSTATE_BASE_H

#include "katanaCommon/katanaCommon.h"
#include "PoseStamped.h"
#include "mission_control/StateBase.h"
#include "mission_control/System.h"
#include "mission_control/DrivingStripChanger.h"
#include "RoadSign.h"


namespace katana
{

class MainStateBase : public StateBase
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<MainStateBase> Ptr;

  //! Constructor
  MainStateBase() = delete;
  MainStateBase(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : StateBase()
    , m_system(system)
    , m_driving_strip_changer(driving_strip_changer)
  {

  }

  //! Destructor
  virtual ~MainStateBase()      {}

  //! Jury pressed start
  virtual void juryRunCommand(int16_t maneuver_entry)  {}

  //! Jury pressed stop
  virtual void juryStopCommand()
  {
    // standard behaviour: change into waiting
    m_change_state((u_int32_t)MainState::WAITING);
  }

  //! Jury requests the state. Return true if ready, false if not
  virtual bool juryRequestState() 			{ return false; }

  //! New pose
  virtual void poseChanged(const PoseWithTime& pose_stamped)    {}

  //! Lanetracker has answered
  virtual bool lanetrackerCallback(u_int32_t job_id)      { return true; }

  //! New patch
  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) {}

  //! New Obstacle
  virtual void newObstacle()			       {}

  //! New Traffic Sign
  virtual void newTrafficSign(RoadSign::Ptr sign, bool signAppeared = true)      {}

  //! Timer
  virtual void timerUpdate()                           {}

  //! Is this an error state and the car can't do more
  virtual bool isError()				{ return false; }

protected:
  //! System
  System::Ptr m_system;

  //! Driving strip
  DrivingStripChanger::Ptr m_driving_strip_changer;
};


} // ns

#endif //_MISSION_CONTROL_MAINSTATE_BASE_H
