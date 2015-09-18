/**
 *
 * ADTF Template Project Filter.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: belkera $
 * $Date: 2011-06-30 16:51:21 +0200 (Thu, 30 Jun 2011) $
 * $Revision: 26514 $
 *
 * @remarks
 *
 */
#ifndef _ODOMETRY_FILTER_H_
#define _ODOMETRY_FILTER_H_


#include "stdafx.h"
#include "Pose.h"
#include "SignalAverage.h"
#include "katanaCommon/katanaCommon.h"
#include "PinConversions.h"

#include <mutex>

#ifdef KATANA_ODOMETRY_DEBUB_PLOT
#include <fstream>
#endif

#define OID_ADTF_ODOMETRY_FILTER "adtf.aadc.odometry" 

using namespace katana;

//*************************************************************************************************
class OdometryFilter : public adtf::cTimeTriggeredFilter
{
  ADTF_FILTER(OID_ADTF_ODOMETRY_FILTER, "KATANA Odometry Filter", adtf::OBJCAT_DataFilter)

public:
  OdometryFilter(const tChar* __info);
  virtual ~OdometryFilter();

  static constexpr const u_int32_t    WHEELBASE = 3600;

  static constexpr const u_int32_t    DISTANCE_PER_TICK = 350;
  //static constexpr const double       MAX_STEERING_VALUE_LEFT = 30;
  //static constexpr const double       MAX_STEERING_ANGLE = -M_PI*28/180;
  //static constexpr const double       STEERING_SENSOR_TO_ANGLE = MAX_STEERING_ANGLE/MAX_STEERING_VALUE_LEFT;
  static constexpr const double       STEERING_SENSOR_TO_ANGLE = -0.01628973969;

  static constexpr const u_int32_t    PUBLISH_RATE = 10;     // ms

protected:

  //! Filter stages
  tResult Init(tInitStage eStage, __exception);
  tResult Shutdown(tInitStage eStage, __exception);

  //! implements IPinEventSink
  tResult OnPinEvent(IPin* pSource,
                     tInt nEventCode,
                     tInt nParam1,
                     tInt nParam2,
                     IMediaSample* pMediaSample);

  tResult Cycle(__exception);

  //! Pins
  cInputPin    m_ipin_wheel_left;
  cInputPin    m_ipin_wheel_right;
  cInputPin    m_ipin_steering;

  cInputPin    m_ipin_status;     //< used to recognize reverse driving

  cOutputPin   m_opin_pose;

  //! Coder Descriptor for the input pins
  cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;


private:
  //! send pose
  tResult sendPose(const katana::Pose& p);

  //! update pose when new wheel tick is present
  void updatePose();

  //! Calculate rear wheel angle
  double getAngleFromSteering(double steering) const;

  //! estimate of current vehicle pose
  katana::Pose m_pose;

  //! Averging of last steering values to smooth signal
  SignalAverage<float, u_int32_t>* m_steering;

  //! Current target speed send to cruise control, used to decide, if vehicle is moving forward or backward
  int32_t m_target_speed;
  bool m_forward;

  //!
  int32_t m_ticks_left;
  int32_t m_ticks_right;

  //!
  int32_t m_current_ticks_left;
  int32_t m_current_ticks_right;

  //!
  u_int32_t m_ignore_counter = 300;

  //!
  u_int32_t m_odometry_error_counter;
  bool m_odometry_error;

  //! Mutex for pose update
  std::mutex m_pose_update_mutex;

#ifdef KATANA_ODOMETRY_DEBUB_PLOT
  std::ofstream m_gnuplot_file;
#endif
};

//*************************************************************************************************
#endif // _ODOMETRY_FILTER_H_
