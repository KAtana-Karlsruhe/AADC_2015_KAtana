#ifndef DRIVEJUNCTION_H
#define DRIVEJUNCTION_H

#include "katanaCommon/katanaCommon.h"
#include "Pose.h"
#include "mission_control/states/driving_states/DriveBase.h"

namespace katana {

enum JUNCTION_STATES : u_int32_t {
  SIGN_FOUND,
  LT_JUNCTION_SEARCH,
  LT_STRAIGHT_SEARCH,
  JUNCTION_FOUND
};

class DriveJunction :  public DriveBase
{
public:

  //! Constructor
  DriveJunction(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : DriveBase(system, driving_strip_changer)
    , m_junction_state(SIGN_FOUND)
    //, m_lt_found_junction(false)
    //, m_notified_lanetracker(false)
    , m_seeing_sign(true)
    , m_lastPoseOfJunctionSet(false)
  {

  }

  //!
  u_int32_t getState() const override   { return (u_int32_t)DrivingState::DRIVE_JUNCTION; }

  //! State activated
  virtual void onActivation(u_int32_t previous) override;

  //! Called by state manager when state will be left, possibility to cancel
  virtual bool onStateLeave(u_int32_t next_state) override;

  //! New patch in world
  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) override;

  //! Timer
  virtual void timerUpdate() override;

  // New Traffic sign
  virtual void newTrafficSign(RoadSign::Ptr sign, bool signAppeared = true) override;

private:

  JUNCTION_STATES m_junction_state;

  Pose2d m_junctionDetectionPose;

  void callLanetrackerJunction();

  void driveForward();

  void junctionFound();

  void driveOverJunction(const World::RoadPatchContainerConst& only_junction);

  void insertVirtualJunctionToWorld();

  RoadBase::Ptr getJunctionGuess();

  Pose2d getStartPoseNextToCar();

  bool missedJunction();

//  bool m_lt_found_junction;

  bool m_notified_lanetracker;

  bool m_seeing_sign;

  bool m_ignoring_obstacles;

  bool m_after_junction_found;

  ExtendedPose2d m_lastPoseOfJunction;
  bool m_lastPoseOfJunctionSet;

  //! Counter to wait at stopline
  u_int8_t m_stopline_wait_counter;
  bool m_waited_at_stopline;

  u_int32_t m_obstacle_stopline_counter;

  Pose2d m_junction_pose;
};

}
#endif // DRIVEPARKING_H
