#ifndef _KATANA_OBSTACLE_FILTER_H_
#define _KATANA_OBSTACLE_FILTER_H_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include <array>

#include "SignalVariance.h"
#include "katanaCommon/katanaCommon.h"
#include "Pose.h"
#include "PinConversions.h"

#define OID_ADTF_OBSTACLE_FILTER "adtf.aadc.obstacle"

using namespace katana;
using namespace adtf;

struct SENSOR_DATA
{
  u_int32_t MIN;
  u_int32_t MAX;
  katana::Pose POSE;
  double VARIANCE;
};

//*************************************************************************************************
class Filter : public adtf::cTimeTriggeredFilter
{
  ADTF_FILTER(OID_ADTF_OBSTACLE_FILTER, "KATANA Obstacle IR/US Filter", adtf::OBJCAT_DataFilter)

public:
  Filter(const tChar* __info);
  virtual ~Filter();

  static constexpr u_int32_t    PUBLISH_RATE = 250;     // ms
  static constexpr u_int32_t    SAMPLE_NUMBER_IR = 5;     //number of samples in Ringbuffer
  static constexpr u_int32_t    SAMPLE_NUMBER_US = 1;     //number of samples in Ringbuffer


  static constexpr u_int32_t IR_SHORT_MIN = 5;
  static constexpr u_int32_t IR_SHORT_MAX = 45;
  static constexpr u_int32_t IR_LONG_MIN = 45;
  static constexpr u_int32_t IR_LONG_MAX = 100;
  static constexpr u_int32_t US_MIN = 20;
  static constexpr u_int32_t US_MAX = 100;

//  static constexpr u_int32_t VARIANCE_TRESHOLD = 10;
  static constexpr double VARIANCE_IR_SHORT = 1;
  static constexpr double VARIANCE_IR_LONG = 20;
  static constexpr double VARIANCE_US = 10;



//  enum SENSORS : u_int8_t
//  {
//    IR_FRONT_CENTER_LONG,
//    IR_FRONT_CENTER_SHORT,
//    IR_FRONT_LEFT_LONG,
//    IR_FRONT_LEFT_SHORT,
//    IR_FRONT_RIGHT_LONG,
//    IR_FRONT_RIGHT_SHORT,
//    IR_REAR_LEFT_LONG,
//    IR_REAR_RIGHT_LONG,
//    IR_REAR_CENTER_SHORT,
//    US_FRONT_LEFT,
//    US_FRONT_RIGHT,
//    US_REAR_LEFT,
//    US_REAR_RIGHT,
//    SENSOR_COUNT = 13
//  };

  static const u_int8_t SENSOR_COUNT = XTION;

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
  cInputPin    m_ipin_ir_front_center_long;
  cInputPin    m_ipin_ir_front_center_short;
  cInputPin    m_ipin_ir_front_left_long;
  cInputPin    m_ipin_ir_front_left_short;
  cInputPin    m_ipin_ir_front_right_long;
  cInputPin    m_ipin_ir_front_right_short;
  cInputPin    m_ipin_ir_rear_left_short;
  cInputPin    m_ipin_ir_rear_right_short;
  cInputPin    m_ipin_ir_rear_center_short;
  cInputPin    m_ipin_us_front_left;
  cInputPin    m_ipin_us_front_right;
  cInputPin    m_ipin_us_rear_left;
  cInputPin    m_ipin_us_rear_right;
  cInputPin    m_ipin_pose;

  cOutputPin   m_opin_obstacles;


  //! Coder Descriptor for the input pins
  cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;


private:
  //!
  void addSample(u_int8_t sensor, float value, u_int32_t stamp);


  //! Current vehicle pose from odometry filter
  katana::Pose m_pose;

  //! Sensor data
  SENSOR_DATA m_sensor_data[SENSOR_COUNT];
  std::array<SignalVariance, SENSOR_COUNT> m_signals;


};

//*************************************************************************************************
#endif // _KATANA_OBSTACLE_FILTER_H_
