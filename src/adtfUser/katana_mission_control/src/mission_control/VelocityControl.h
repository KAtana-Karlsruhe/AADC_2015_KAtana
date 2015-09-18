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

#ifndef _MISSION_CONTROL_VELOCITY_CONTROL_H
#define _MISSION_CONTROL_VELOCITY_CONTROL_H

#ifdef __x86_64__
// Controller has problems when testing with completely unrealistic odometry simulator, so disable when not on AADC-Car
#define MC_DISABLE_PID_LONGITUDINAL_CONTROLLER
#endif

#include "katanaCommon/katanaCommon.h"
#include "SignalAverage.h"

#include "helperClasses/Config.h"

#include <oadrive_core/Pose.h>
#include <oadrive_core/Trajectory2d.h>

#include <icl_core/DataHeader.h>

#include <PoseWithTime.h>

using namespace oadrive::core;

namespace katana
{

class VelocityControl
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<VelocityControl> Ptr;

  //! Constructor
  VelocityControl();

  //! Destructor
  virtual ~VelocityControl();

  void initialize(Config::Ptr config = nullptr);

  //! Cut-off speed threshold when controller assumes vehicle is not moving
  static constexpr double VEHICLE_ZERO_SPEED_THRESHOLD = 0.005;

  //! Gets updated by mission control (independent by current state)
  void poseChanged(const PoseWithTime &p);

  //! Read flag
  bool isBlocked() const    { return m_blocked; }
  //! try to drive again
  void setNonBlocked()      { m_blocked = false; }

  void resetPID()
  {
    m_pid_sum = 0;
  }

  //! Let this be called regularly (10 Hz?) -> check for speed and position on driving strip
  //! Current point index from lateral Controller and vehicle pose needed
  int8_t longControllerUpdate(double target_velocity)
  {
    return controller(target_velocity);
  }

  //! This overload ensures that the vehicle will not move
  int8_t longControllerUpdate()
  {
    return controller(0.0);
  }

  //! set driving direction
  void setDrivingDirection(bool forward = true)
  {
    if (m_forward != forward)
    {
      m_forward = forward;

      // reset PID controller
      m_pid_sum = 0.0;
      m_pid_last_error = 0.0;
    }
  }

private:
  //! PID controller, returns torque
  float controller(float target);

  //!
  PoseWithTime m_last_pose;
  bool m_last_pose_available;

  //! The current speed, calculated in the controller(...) function
  float m_speed;

  //! queue of last vehicle poses for speed calculation
  SignalAverage<double, _time_type>* m_distance_queue;

  //! Flag if vehicle is blocked (or arduino down, etc.)
  bool m_blocked;

  //! Flag for driving direction
  bool m_forward;

  //! PID-Controller
  float m_pid_sum;
  float m_pid_last_error;
  u_int32_t m_has_car_stopped_counter;

  //! Controller constants
  Config::Ptr m_config;

#ifdef MC_VELOCITY_READ_PARAM_FROM_FILE
  float KI = 0.0027;
  float KD = 0.0004;
  float KP = 0.0013;

  float KI_TA;
  float KD__TA;

#else
  //! Sampling time
  static const constexpr float TA = 0.07;

  static const constexpr float KP = 30;
  //static const constexpr float KI = 0.0027;
  //static const constexpr float KD = 0.0004;

  //static const constexpr float KI_TA = KI*TA;
  //static const constexpr float KD__TA = KD/TA;
  static const constexpr float KI_TA = 1.4;
  static const constexpr float KD__TA = 57.14;
#endif

  float m_ta;
  float m_kp;
  float m_ki_ta;
  float m_kd_ta;

  static const constexpr float FEEDFORWARD = 65.0;
  static const constexpr float FEEDFORWARD_HYSTERESIS = 15.0;

  // time when controller assumes there is a physical error in the power train
  static const constexpr float MAX_ALLOWED_ERROR_SECONDS = 10.0;
  u_int32_t m_car_blocked_counter;

  // set this to <100 as a security measure
  static const constexpr int8_t MAX_OUTPUT = 50;

  static const constexpr u_int32_t SPEED_CALCULATION_INTERVAL_MICROSECONDS = 500000;
};


} // ns

#endif //_MISSION_CONTROL_VELOCITY_CONTROL_H
