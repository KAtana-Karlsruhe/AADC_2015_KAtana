// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-06
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_POSITION_CONTROLLER_H
#define _MISSION_CONTROL_POSITION_CONTROLLER_H

#include <thread>
#include <chrono>
#include <mutex>

#include "katanaCommon/katanaCommon.h"

#include "mission_control/VelocityControl.h"
#include "mission_control/LateralController.h"
#include "mission_control/LightController.h"

#include <queue>

namespace katana
{

//! Send torque and steering
typedef std::function<void(float)> FuncSteering;
typedef std::function<void(float)> FuncTorque;

//! Status transmit function
typedef std::function<void(MissionControlStatus status)> TransmitStatusFunc;

//! Different independent stop flags
enum StopReason : u_int8_t
{
  JURY=0,
  OBSTACLE=1,
  MANEUVER_COMPLETE=2,
  INITIALIZING=3,
  JUNCTION=4,
  JUNCTION_NOT_FOUND=5,
  COUNT_INTERNAL=6      //< keep up-to-date!
};

/**
 * @brief The PositionController class
 * Ensures that the Car drives along the given trajectories (saved in queue)
 */
class PositionController
{
public:
  typedef std::array<bool, (u_int32_t)StopReason::COUNT_INTERNAL> StopFlags;

  //! Trajectory container
  typedef std::queue<Trajectory2d> TrajectoryQueue;

  //! Convenience pointer
  typedef std::shared_ptr<PositionController> Ptr;
  typedef std::shared_ptr<const PositionController> ConstPtr;

  //! Constructor
  PositionController() = delete;
  PositionController(FuncSteering send_steering, FuncTorque send_torque, TransmitStatusFunc send_status);

  //! No copying
  PositionController(const PositionController& rs) = delete;
  PositionController& operator=(const PositionController& rhs) = delete;

  //! Destructor
  virtual ~PositionController();

  //! Set light controller
  void initialize(const LightController::Ptr& light_controller, const Config::Ptr& config)
  {
    m_light_controller = light_controller;
    m_config = config;

    m_longitudinal_controller.initialize(config);


    // start execution of controller thread
    m_controller_thread = std::make_shared<std::thread>(&PositionController::controllerUpdate, this);
  }

  //! Read flags
  //! When currently switching driving direction, reached is false
  bool hasReached() const   { return m_lateral_controller.hasReached() && (m_switching_driving_direction == 0); }
  bool mustNotMove() const  { return m_must_not_move; }
  bool isBlocked() const    { return m_longitudinal_controller.isBlocked(); }
  void setNonBlocked()      { m_longitudinal_controller.setNonBlocked(); }

  //! get current steering angle
  float getCurrentSteering() const
  {
    std::lock_guard<std::mutex> lock(m_controller_mutex);
    return m_current_steering;
  }

  //! Read custom stop flags
  bool mustNotMove(StopReason stop_reason)      { return m_stop_flags[stop_reason]; }

  //! Read current pose
  const Pose2d& getCurrentVehiclePose() const     { return m_pose_with_time.pose; }
  //! Get overall driven distance
  const double& getOverallDrivenDistance() const         { return m_overall_driven_distance; }

  //! get projected pose
  const ExtendedPose2d& getProjectedPose() const      { return m_lateral_controller.getProjectedPose(); }
  std::size_t getProjectedPoseIndex() const           { return m_lateral_controller.getIndexOfProjectedPose(); }

  //! Read and write to current Trajectory
  const oadrive::core::Trajectory2d& getTrajectory() const             { return m_lateral_controller.getTrajectory(); }
  oadrive::core::Trajectory2d& getTrajectory()                         { return m_lateral_controller.getTrajectory(); }

  //! Let this be called on any vehicle pose update
  void poseChanged(const PoseWithTime& pose)
  {
    // take care of overall distance
    if (m_overall_driven_distance == std::numeric_limits<double>::infinity())   //< initial state, no last pose available
      m_overall_driven_distance = 0.0;
    else
      m_overall_driven_distance += (pose.pose.translation() - m_pose_with_time.pose.translation()).norm();

    m_pose_with_time = pose;
    m_longitudinal_controller.poseChanged(pose);
  }

  //! STOP as global override
  void stop(StopReason reason);
  //! Drive
  void go(StopReason reason);

  //! Set trajectory
  void setTrajectory(const Trajectory2d& trajectory);
  void clearTrajectory()
  {
    m_controller_mutex.lock();
    m_lateral_controller.getTrajectory().clear();
    m_controller_mutex.unlock();
  }

  //! Check position if reached
  void setPositionToCheck(REACHED_POINT rp_type, const oadrive::core::Position2d& position_to_check, double reached_distance = 0.02)
                    { m_lateral_controller.setPositionToCheck(rp_type, position_to_check, reached_distance); }
  bool hasReachedPosition(REACHED_POINT rp_type) const   { return m_lateral_controller.hasReachedPosition(rp_type); }

  //! Return the nearest pose on current trajectory
  oadrive::core::Pose2d getNearestPoseOnCurrentTrajectory(const oadrive::core::Position2d& position) const;
  std::size_t getNearestIndexOnTrajectory(const oadrive::core::Trajectory2d& t, const oadrive::core::Position2d& position) const;

  //!
  oadrive::core::Position2d getStopLineReachedPose(const Pose2d& junction_start_pose) const;

private:
  /** ********** UPDATE **********/
  //! lateral controller update (100 Hz)
  void lateralControllerUpdate();

  //! longitudinal controller update (20 Hz)
  void longitudinalControllerUpdate();

  //! Thread which ensures the controller updates
  void controllerUpdate();
  std::shared_ptr<std::thread> m_controller_thread;
  mutable std::mutex m_controller_mutex;

  //! Flag to stop the controller thread (volatile: ensure that the compiler re-reads this value)
  volatile bool m_running;


  //! Ensure that the vehicle stays in position
  void holdVehicle();

  //! Current vehicle pose
  PoseWithTime m_pose_with_time;

  //! Overall driven distance (forward and backward counted as positive)
  double m_overall_driven_distance;

  //! Move reached points onto new trajectory when updating trajectory
  void moveReachedPoints();

  /** ****** CONTROLLER ******* **/
  //! Config object
  Config::Ptr m_config;

  //! Longitudinal controller
  VelocityControl m_longitudinal_controller;
  //! Lateral controller
  LateralController m_lateral_controller;

  //! Function pointer to send steering angle
  FuncSteering m_send_steering;
  //! Function pointer to send torque
  FuncTorque m_send_torque;

  //! Current steering angle
  volatile float m_current_steering;

  //! Function pointer to send reverse status
  TransmitStatusFunc m_send_status;

  //! Check if projection on trajectory is under car, if not, interpolate linear
  void expandTrajectory(oadrive::core::Trajectory2d& trajectory) const;

  //! Flags
  bool m_must_not_move;     //!< if set to true, the vehicle must not move
  StopFlags m_stop_flags;

  //! sets the m_must_not_move_flag
  void updateStopFlags()
  {
    for (size_t i = 0; i < m_stop_flags.size(); i++)
    {
      if (m_stop_flags[i])
      {
        m_must_not_move = true;
        m_light_controller->setLight(LightName::BREAK_LIGHT, true);
        return;
      }
    }
    m_must_not_move = false;
    m_light_controller->setLight(LightName::BREAK_LIGHT, false);
  }

  //! This is set to !=0 when we are changing the driving direction, ensure that the vehicle stands still
  u_int32_t m_switching_driving_direction;

  //! Counter so send steering angle with less than controller rate
  u_int8_t m_steering_frequency_counter;

#ifdef KATANA_MC_POSITION_CONTROLLER_DEBUG_PLOT
  u_int32_t trajectory_write_counter = 0;
#endif

  //! Trajectory to save trajectory with other driving direction while vehicle is slowing down
  Trajectory2d m_change_direction_trajectory;

  //! LightController (brake light)
  LightController::Ptr m_light_controller;
};


} // ns

#endif //_MISSION_CONTROL_POSITION_CONTROLLER_H
