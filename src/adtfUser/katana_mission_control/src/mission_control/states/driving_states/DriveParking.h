#ifndef DRIVEPARKING_H
#define DRIVEPARKING_H

#include "katanaCommon/katanaCommon.h"
#include "Pose.h"
#include "mission_control/states/driving_states/DriveBase.h"

#include <chrono>

namespace katana {

class DriveParking :  public DriveBase
{
public:

  enum ParkingState : u_int8_t
  {
    SEARCHING,
    FORWARD,
    REVERSING,
    FORWARD2,
    WAITING
  };

  static constexpr double CHECK_FOR_SPOT_DISTANCE = 0.1;

  //! Waiting time after parking is finished
  static constexpr auto WAITING_DURATION = std::chrono::milliseconds(3500);

  //! Constructor
  DriveParking(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : DriveBase(system, driving_strip_changer)
    , m_state(ParkingState::SEARCHING)
  {

  }

  //!
  u_int32_t getState() const override   { return (u_int32_t)DrivingState::DRIVE_PARKING; }

  //! State activated
  virtual void onActivation(u_int32_t previous) override;

  //! Called by state manager when state will be left, possibility to cancel
  virtual bool onStateLeave(u_int32_t next_state) override;

  //! Check in timer if already parked
  virtual void timerUpdate() override;

  //! Trigger lanetracker, same as normal driving
  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) override;

private:

  //! Let the car back up into cross parking space immediately, sets parking patch at patch_pose
  void reversePark(const oadrive::core::Pose2d& patch_pose, ParkingSpace type);
  void forwardMove(const oadrive::core::Pose2d& patch_pose, ParkingSpace type);

  //! Calculate pose to place parking patch
  oadrive::core::Pose2d getParkingPatchPose(const double& diff_distance) const;

  //! Parking assistant returned from world
  ParkingAssistant::Ptr m_parking_assistant;

  //! Last distance checked for parking spot
  double m_distance_checked;

  //! Flag for state
  ParkingState m_state;

  //! Waiting
  std::chrono::steady_clock::time_point m_waiting_start;

  //! Pose where to place parking patch
  oadrive::core::Pose2d m_patch_pose;

};

}
#endif // DRIVEPARKING_H
