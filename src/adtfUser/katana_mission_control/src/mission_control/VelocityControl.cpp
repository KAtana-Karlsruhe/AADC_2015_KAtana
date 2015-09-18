// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-21
 *
 */
//----------------------------------------------------------------------

#include "mission_control/VelocityControl.h"
#include <cmath>

#ifdef KATANA_MC_VELOCITY_DEBUG
#include <iostream>
#endif

#include <fstream>

namespace katana
{

VelocityControl::VelocityControl()
  : m_last_pose_available(false)
  , m_speed(0)
  , m_blocked(false)
  , m_forward(true)
  , m_pid_sum(0)
  , m_pid_last_error(0)
  , m_has_car_stopped_counter(0)
  , m_car_blocked_counter(0)
{
  m_distance_queue = new SignalAverage<double, katana::_time_type>(SPEED_CALCULATION_INTERVAL_MICROSECONDS);

}

void VelocityControl::initialize(Config::Ptr config)
{
  m_config = config;

  if (m_config == nullptr)
  {
    m_ta = TA;
    m_kp = KP;
    m_ki_ta = KI_TA;
    m_kd_ta = KD__TA;
  }
  else
  {
    m_ta = config->getDouble(CONF_PID_TA);
    m_kp = config->getDouble(CONF_PID_KP);
    m_ki_ta = config->getDouble(CONF_PID_KI) * m_ta;
    m_kd_ta = config->getDouble(CONF_PID_KD) / m_ta;

  }
}

VelocityControl::~VelocityControl()
{
  delete m_distance_queue;
}


void VelocityControl::poseChanged(const PoseWithTime &p)
{
  if (m_last_pose_available)
  {
    m_distance_queue->addSample((p.pose.translation() - m_last_pose.pose.translation()).norm(), p.time);
  }
  m_last_pose = p;
  m_last_pose_available = true;
}


float VelocityControl::controller(float target)
{
  // calculate current speed
  if (m_distance_queue->getContainerSize() > 1)
  {
    _time_type t_diff = m_distance_queue->getNewest().second - m_distance_queue->getOldest().second;

    m_speed = m_distance_queue->getSum() * 1000000 / t_diff * (m_forward ? 1.0 : -1.0); //< m per second

  }
  else
    m_speed = 0.0;

#ifdef KATANA_MC_VELOCITY_DEBUG
  std::cout <<"VelocityControl: controller - current Speed: " <<m_speed <<" Target: " <<target <<std::endl;
#endif

  // reset PID if vehicle has to stop for a certain time...
  // Check if car should have stopped
  if (target == 0)
  {
    if (m_has_car_stopped_counter < 10)
    {
      ++m_has_car_stopped_counter;
    }
    else
    {
      m_pid_sum = 0;
      m_pid_last_error = 0;
    }
  }
  else
  {
    m_has_car_stopped_counter = 0;
  }

  // car is blocked
  if (m_blocked)
  {
    // reset PID controller
    m_pid_last_error = 0;
    m_pid_sum = 0;
    // reset counter
    m_car_blocked_counter = 0;
    return 0.0;   //< do not attempt to drive
  }

  // check if vehicle is blocked
  if (std::abs(m_speed) < VEHICLE_ZERO_SPEED_THRESHOLD && std::abs(target) > VEHICLE_ZERO_SPEED_THRESHOLD)
  {
    if (m_car_blocked_counter < MAX_ALLOWED_ERROR_SECONDS/m_ta)
    {
      ++m_car_blocked_counter;
    }
    else
    {
      m_blocked = true;
    }
  }
  if (std::abs(m_speed) >= VEHICLE_ZERO_SPEED_THRESHOLD || std::abs(target) <= VEHICLE_ZERO_SPEED_THRESHOLD)
  {
    m_car_blocked_counter = 0;
  }

  // Feedforward to improve controller
  float output = FEEDFORWARD * target;
  if (target > 0.0)
  {
    output += FEEDFORWARD_HYSTERESIS;
  }
  else if (target < 0.0)
  {
    output -= FEEDFORWARD_HYSTERESIS;
  }

#ifdef MC_DISABLE_PID_LONGITUDINAL_CONTROLLER
  // disable PID controller, just use fixed proportional value as output
#ifdef KATANA_MC_VELOCITY_DEBUG
    std::cout <<"VelocityControl: controller output speed: " <<output <<std::endl;
#endif
  return output;
#endif

  //******* PID controller to control error ***********/
  float e = target - m_speed;
  m_pid_sum += e;

  float control_pid = m_kp * e + m_ki_ta * m_pid_sum + m_kd_ta * (e - m_pid_last_error);

  m_pid_last_error = e;
  /*****************************************************/

  // PID as feedforward error correction
  output += control_pid;

  if (output > MAX_OUTPUT)
    output = MAX_OUTPUT;
  else if (output < -MAX_OUTPUT)
    output = -MAX_OUTPUT;

  // the feedback loop of the vehicle (odometry) does not recognize driving direction
  // -> to avoid absolut instability of the controller, only send positiv outputs on forward trajectory and vice versa
  if ((output < 0.0) == m_forward)
    output = 0.0f;

#ifdef KATANA_MC_VELOCITY_DEBUG
  std::cout <<"VelocityControl: controller output speed: " <<output <<" | PID: " <<control_pid <<std::endl;
#endif
  return output;
}

} // ns
