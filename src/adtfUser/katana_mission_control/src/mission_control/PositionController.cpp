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

#include "mission_control/PositionController.h"

#include <oadrive_core/Interpolator.h>

#ifdef KATANA_MC_POSITION_CONTROLLER_DEBUG
#include <iostream>
#endif

using namespace oadrive::core;

namespace katana
{

PositionController::PositionController(FuncSteering send_steering, FuncTorque send_torque, TransmitStatusFunc send_status)
  : m_running(true)
  , m_overall_driven_distance(std::numeric_limits<double>::infinity())
  , m_send_steering(send_steering)
  , m_send_torque(send_torque)
  , m_send_status(send_status)
  , m_must_not_move(true)
  , m_switching_driving_direction(0)
  , m_steering_frequency_counter(0)
{
  // init stop flags
  for (size_t i = 0; i < m_stop_flags.size(); i++)
    m_stop_flags[i] = false;

  // better stop at beginning
  m_stop_flags[StopReason::JURY] = true;
}

PositionController::~PositionController()
{
  // signal controller thread to stop
  m_running = false;
  // wait for thread to finish execution
  if (m_controller_thread)
    m_controller_thread->join();
}

void PositionController::lateralControllerUpdate()
{

  // lock reading of m_trajectories
  std::lock_guard<std::mutex> lock(m_controller_mutex);

  if (!m_lateral_controller.getTrajectory().empty() && !m_lateral_controller.hasReached())
    m_current_steering = m_lateral_controller.calculateSteering(m_pose_with_time.pose);
  else
    m_current_steering = 0.0;

  // send steering only with half of controller rate
  if (m_steering_frequency_counter % 2)
  {
    m_steering_frequency_counter = 0;
    m_send_steering(m_current_steering);
  }
  else
    ++m_steering_frequency_counter;

}

void PositionController::longitudinalControllerUpdate()
{
  // lock reading of m_trajectories
  std::lock_guard<std::mutex> lock(m_controller_mutex);

  // this ensures that we are stopping before driving reverse (and vice versa)
  if (m_switching_driving_direction > 0)
  {
    // last step, vehicle is probably standing, now set longitudinal controller to direction, and switch odometry
    if (m_switching_driving_direction == 1)
    {
      m_longitudinal_controller.resetPID();

      const bool forward = m_change_direction_trajectory.isForwardTrajectory();

#ifdef KATANA_REVERSE_DRIVING_DEBUG
      std::cout <<"MissionControl - PositionController - Now switching odometry and long controller to " <<(forward ? "forward" : "reverse") <<std::endl;
#endif

      // NOW set trajectory of opposite driving direction
      m_lateral_controller.setTrajectory(m_change_direction_trajectory);

      // if lateral controller needs to check for reached, move this position onto the new trajectory
      moveReachedPoints();

      m_longitudinal_controller.setDrivingDirection(forward);
      m_send_status(forward ? MissionControlStatus::DRIVING_FORWARD : MissionControlStatus::DRIVING_REVERSE);


#ifdef KATANA_MC_POSITION_CONTROLLER_DEBUG_PLOT
      m_lateral_controller.getTrajectory().toGnuplot(std::string("/tmp/PositionController") + std::to_string(trajectory_write_counter + 1000));
      ++trajectory_write_counter;
#endif
    }

    --m_switching_driving_direction;
    holdVehicle();
    return;
  }

  // stop car if at end of trajectory of shall not move
  if (m_lateral_controller.hasReached() || m_must_not_move)
  {
    holdVehicle();
    return;
  }

  // limit velocity dependant of steering
  double desired_velocity = m_lateral_controller.getProjectedPose().getVelocity();

  const double st = std::abs(m_current_steering) - 100.0;
  const double max = 0.20 + (st*st)*0.0035;

  if (desired_velocity > max)
    desired_velocity = max;

  m_send_torque(m_longitudinal_controller.longControllerUpdate(desired_velocity));
}

void PositionController::holdVehicle()
{
  // let the vehicle stand still!
  m_send_torque(m_longitudinal_controller.longControllerUpdate());
}

void PositionController::controllerUpdate()
{
  //! used to run longitudinal controller at slower frequency
  static u_int8_t freq_counter = 0;

  while (m_running)
  {

    // Get start time
    const auto start_time = std::chrono::steady_clock::now();
    // Get end time
    const auto end_time = start_time + std::chrono::milliseconds(20);

    // call lateral controller
    lateralControllerUpdate();

    if (freq_counter % 3 == 0)
    {
      freq_counter = 0;

      // call long controller
      longitudinalControllerUpdate();

    }
    freq_counter++;

    // Sleep if necessary
    std::this_thread::sleep_until(end_time);

  }
}

void PositionController::setTrajectory(const oadrive::core::Trajectory2d& trajectory)
{
  {
    assert(trajectory.size() >= MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER);
    assert(trajectory.curvatureAvailable());
    assert(trajectory.velocityAvailable());

    if (trajectory.isForwardTrajectory() != m_lateral_controller.getTrajectory().isForwardTrajectory())
    {
#ifdef KATANA_REVERSE_DRIVING_DEBUG
      std::cout <<"MissionControl - PositionController - Initiaiting driving direction change" <<std::endl;
#endif

      // delay setting of new trajectory until vehicle has stopped
      m_change_direction_trajectory = trajectory;
      m_switching_driving_direction = 15;   //< 22* 1s/16

      // lights
      m_light_controller->setLight(LightName::REVERSE_LIGHT, !trajectory.isForwardTrajectory());
    }
    else
    {
      m_controller_mutex.lock();
      m_lateral_controller.setTrajectory(trajectory);


      // check if needs to be expanded
      expandTrajectory(m_lateral_controller.getTrajectory());

      m_controller_mutex.unlock();

      // if lateral controller needs to check for reached, move this position onto the new trajectory
      moveReachedPoints();

#ifdef KATANA_MC_POSITION_CONTROLLER_DEBUG_PLOT
      m_lateral_controller.getTrajectory().toGnuplot(std::string("/tmp/PositionController") + std::to_string(trajectory_write_counter + 1000));
      ++trajectory_write_counter;
#endif
    }



#ifdef KATANA_MC_POSITION_CONTROLLER_DEBUG
    std::cout <<"MissionControl - PositionController - SetTrajectory" <<std::endl;
#endif

  }
}

void PositionController::go(StopReason reason)
{
#ifdef KATANA_MC_POSITION_CONTROLLER_DEBUG
  std::cout <<"MissionControl - PositionController - GO, reason: " <<(u_int32_t)reason <<std::endl;
#endif
  m_stop_flags[reason] = false;
  updateStopFlags();
}

void PositionController::stop(StopReason reason)
{
  assert(reason < StopReason::COUNT_INTERNAL && "reason < StopReason::COUNT_INTERNAL");

#ifdef KATANA_MC_POSITION_CONTROLLER_DEBUG
  std::cout <<"MissionControl - PositionController - STOP, reason: " <<(u_int32_t)reason <<std::endl;
#endif
  m_stop_flags[reason] = true;
  updateStopFlags();
}

Pose2d PositionController::getNearestPoseOnCurrentTrajectory(const Position2d& position) const
{
  assert(!m_lateral_controller.getTrajectory().empty() && "!m_lateral_controller.getTrajectory().empty()");

  double min_distance = std::numeric_limits<double>::infinity();
  size_t index = 0;

  for (size_t i = 0; i < m_lateral_controller.getTrajectory().size(); i++)
  {
    const double dist = (m_lateral_controller.getTrajectory()[i].getPosition() - position).norm();
    if (dist < min_distance)
    {
      min_distance = dist;
      index = i;
    }
  }

  return (m_lateral_controller.getTrajectory()[index]).getPose();
}

std::size_t PositionController::getNearestIndexOnTrajectory(const oadrive::core::Trajectory2d& t, const Position2d& position) const
{
  assert(!t.empty() && "!m_lateral_controller.getTrajectory().empty()");

  double min_distance = std::numeric_limits<double>::infinity();
  size_t index = 0;

  for (size_t i = 0; i < t.size(); i++)
  {
    const double dist = (t[i].getPosition() - position).norm();
    if (dist < min_distance)
    {
      min_distance = dist;
      index = i;
    }
  }

  return index;
}

void PositionController::moveReachedPoints()
{
  ExtendedPose2d projection;
  double distance;
  std::size_t index;

  // if lateral controller needs to check for reached, move this positions onto the new trajectory
  for (u_int8_t i = 0; i < (u_int8_t)REACHED_POINT::RP_COUNT; i++)
  {
    if (!m_lateral_controller.hasReachedPosition((REACHED_POINT)i))
    {
      m_lateral_controller.calculateProjection(m_lateral_controller.getTrajectory(),
                                               m_lateral_controller.getPositionToCheck((REACHED_POINT)i),
                                               projection,
                                               distance,
                                               index);

      m_lateral_controller.setPositionToCheck((REACHED_POINT)i, projection.getPose().translation(), m_lateral_controller.reachedDistanceToCheck((REACHED_POINT)i));
    }
  }
}

void PositionController::expandTrajectory(Trajectory2d& trajectory) const
{
  ExtendedPose2d projection;
  std::size_t pose_index;
  double distance;

  // let lateral controller check
  if (m_lateral_controller.calculateProjection(trajectory, m_pose_with_time.pose.translation(), projection, distance, pose_index))    //< projection before beginning of pose
  {
#ifdef KATANA_MC_DEBUG
    std::cout <<"[PositionController] Linear expanding trajectory under car." <<std::endl;
#endif

    const double distance = (projection.getPosition() - trajectory[0].getPosition()).norm();
    const size_t number = distance/0.04;

    Trajectory2d expansion;
    expansion.push_back(projection);

    for (size_t i = 1; i < number; i++)
    {
      const double ratio = (double)i/number;
      const ExtendedPose2d inter = Interpolator::interpolateLinear(projection, trajectory.front(), ratio);
      expansion.push_back(inter);
    }

    expansion.append(trajectory);
    trajectory = expansion;
    trajectory.curvatureAvailable() = true;
    trajectory.velocityAvailable() = true;
  }

}

Position2d PositionController::getStopLineReachedPose(const Pose2d& junction_start_pose) const
{
  Pose2d diff;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(diff, -0.6, -0.23, 0.0);

  const Pose2d stop_pose = junction_start_pose * diff;

  ExtendedPose2d projection;
  double distance;
  std::size_t index;

  m_lateral_controller.calculateProjection(m_lateral_controller.getTrajectory(), stop_pose.translation(), projection, distance, index);

  return projection.getPosition();
}


} // ns
