// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-10
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_DRIVEPULLOUT_H
#define _MISSION_CONTROL_DRIVEPULLOUT_H

#include "katanaCommon/katanaCommon.h"
#include "mission_control/states/driving_states/DriveBase.h"

#include <chrono>

namespace katana
{

class DrivePullOut : public DriveBase
{
public:

  //! Waiting time until obstacles have been added
  static constexpr auto WAITING_DURATION = std::chrono::milliseconds(1000);

  enum PullOutState : u_int8_t
  {
    WAITING_FOR_OBSTACLES,
    WAITING_FOR_VISION_TO_CHECK,        //< only when parking spot type not otherwise known
    LOCALIZE_PARALLEL_STREET,           //< only when parallel parking
    PULL_OUT_REVERSE,                   //< only when parallel parking
    WAITING_FOR_LANETRACKER_TO_DRIVE,
    DRIVING
  };

  //! Constructor
  DrivePullOut(const System::Ptr& system, const DrivingStripChanger::Ptr driving_strip_changer)
    : DriveBase(system, driving_strip_changer)
    , m_state(PullOutState::WAITING_FOR_OBSTACLES)
    , m_space(ParkingSpace::PARKING_SPACE_TYPE_COUNT)
  {

  }

  //!
  u_int32_t getState() const override   { return (u_int32_t)DrivingState::DRIVE_PULLOUT; }

  /**
   * @brief onActivation
   * Check maneuver file for action
   * Initialise Vision with appropriate first search patch
   * (currently: there is no special patch to pull out of parking spot -> let lateral controller try)
   */
  virtual void onActivation(u_int32_t previous) override;

  //! Called by state manager when state will be left, possibility to cancel
  virtual bool onStateLeave(u_int32_t next_state) override;

  //! Receive match values for parking space types
  virtual bool lanetrackerCallback(u_int32_t job_id) override;

  /**
   * @brief newPatch
   * wait for answer from lanetracker, if lanetracker has seen nothing, try again... @todo change searchspace, drive blind, etc...
   */
  virtual void newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id) override;

  //! Check if already on street, then switch to drive normal
  void timerUpdate() override;

private:

  //! Send search patch to vision
  void sendSearchPatchToVision();

  //! Check maneuver file, obstacles, etc. to gather information about vehicles position relative to street
  void getEstimatedStartPose(oadrive::core::Pose2d& p, ParkingSpace type, Action direction) const;

  //! Try to match different parking space types
  void tryParkingSpaces();
  u_int32_t localizeStreet(ParkingSpace type);
  ParkingSpace parkingSpaceByVision(double match_value, ParkingSpace type);
  double m_best_matching_value;
  ParkingSpace m_best_option;
  u_int8_t m_answers_from_lanetracker;
  u_int32_t m_test_job_ids[ParkingSpace::PARKING_SPACE_TYPE_COUNT];

  //! Calculate pose of pull out patch when parallel parking
  void calculatePullOutPatchPose(oadrive::core::Pose2d& patch_pose, const oadrive::core::Pose2d street_patch_localization) const;

  //! Localize parallel street
  u_int32_t m_job_localize_patch;

  //! Pose for pull out patch
  oadrive::core::Pose2d m_pull_out_pose;

  //! Pose of localized street (parallel parking
  oadrive::core::Pose2d m_parallel_street_pose;
  //!
  double m_yaw_to_street;

  //! after type of parking spot has been determined, start blinking, let lanetracker search for street
  void beginSearchingForStreet();

  //! Determine the kind of parking spot, return true if cross_parking
  ParkingSpace getParkingSpaceTypeByObstacle() const;

  //! Start turn signals
  void turnOnTurnSignals();

  //! state
  PullOutState m_state;

  //! Type of parking space
  ParkingSpace m_space;

  //! True if reached pose has been set and state is waiting for drive normal
  oadrive::core::Pose2d m_reached_pose;

  //! Waiting
  std::chrono::steady_clock::time_point m_waiting_start;

  //!
  static const oadrive::core::Position2d STRAIGHT;
};


} // ns

#endif //_MISSION_CONTROL_DRIVEPULLOUT_H
