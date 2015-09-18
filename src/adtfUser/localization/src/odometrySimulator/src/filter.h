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
#ifndef _ODOMETRY_SIMULATOR_FILTER_H_
#define _ODOMETRY_SIMULATOR_FILTER_H_


#include "stdafx.h"
#include "Pose.h"
#include "katanaCommon/katanaCommon.h"
#include "PinConversions.h"

#define OID_ADTF_ODOMETRY_SIMULATOR_FILTER "adtf.aadc.odometrysimulator"

using namespace katana;

//*************************************************************************************************
class OdometrySimulatorFilter : public adtf::cTimeTriggeredFilter
{
  ADTF_FILTER(OID_ADTF_ODOMETRY_SIMULATOR_FILTER, "KATANA Odometry Simulator Filter", adtf::OBJCAT_DataFilter)

public:
  OdometrySimulatorFilter(const tChar* __info);
  virtual ~OdometrySimulatorFilter();

  //static constexpr const float        MAX_STEERING_VALUE_LEFT = 50;
  //static constexpr const float        MAX_STEERING_ANGLE = (float)-28/180*M_PI;
  //static constexpr const float        STEERING_SENSOR_TO_ANGLE = MAX_STEERING_ANGLE/MAX_STEERING_VALUE_LEFT;
  static constexpr const double       STEERING_SENSOR_TO_ANGLE = -0.01628973969;

  static constexpr const float        WHEELBASE = 3600;

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
  cInputPin    m_ipin_steering;
  cInputPin    m_ipin_speed_cmd;

  cOutputPin   m_opin_pose_buffer;

  //! Coder Descriptor for the input pins
  cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;

  ///////////// TEST ///////////////
  cInputPin m_ipin_patches;


private:
  //! send pose
  tResult sendPose(const katana::Pose& p);

  //!
  float getDistanceInInterval(float current_speed);

  //! update pose when new wheel tick is present
  void updatePose();

  //! Calculate rear wheel angle
  double getAngleFromSteering(double steering) const;

  //! estimate of current vehicle pose
  katana::Pose m_pose;

  //!
  float m_current_steering;
  float m_current_speed;
  bool m_forward;

  //!
  int8_t m_steering_drift;

  //!
  u_int32_t m_publish_rate;

};

//*************************************************************************************************
#endif // _ODOMETRY_SIMULATOR_FILTER_H_
