/**
 *
 * ADTF Template Project
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
#include "stdafx.h"
#include "filter.h"

//#define DEBUG_OUTPUT

#ifdef KATANA_ODOMETRY_SIMULATOR_DEBUG
#include <iostream>
#endif
#ifdef KATANA_ODOMETRY_SIMULATOR_PATCH_INPUT_DEBUG
#include <iostream>
#endif

/// Create filter shell
ADTF_FILTER_PLUGIN("AADC Odometry Simulator Filter", OID_ADTF_ODOMETRY_SIMULATOR_FILTER, OdometrySimulatorFilter)


OdometrySimulatorFilter::OdometrySimulatorFilter(const tChar* __info)
  : cTimeTriggeredFilter(__info)
  , m_current_steering(0)
  , m_current_speed(0)
  , m_forward(true)
  , m_steering_drift(0)
  , m_publish_rate(PUBLISH_RATE)
{
  //LOG_INFO(adtf_util::cString::Format("counter %d...%d , rpm = %f ",counterValue,lastCounterValue,rpm));

  SetPropertyInt("Publish_rate_ms", PUBLISH_RATE);
  SetPropertyInt("Steering_drift", m_steering_drift);
}

OdometrySimulatorFilter::~OdometrySimulatorFilter()
{

}
/**
 *   The Filter Init Function.
 *    eInitStage ... StageFirst ... should be used for creating and registering Pins
 *               ... StageNormal .. should be used for reading the properies and initalizing
 *                                  everything before pin connections are made
 *   see {@link IFilter#Init IFilter::Init}.
 *
 */
tResult OdometrySimulatorFilter::Init(tInitStage eStage, __exception)
{
  // never miss calling the parent implementation!!
  RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr))
    
  // in StageFirst you can create and register your static pins.
  if (eStage == StageFirst)
  {
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    const tChar* strDescSignalValue;
    // get a media type for the input pins
    strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput));
        
    // create and register the input pins
    RETURN_IF_FAILED(m_ipin_steering.Create("steering_angle", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_steering));
    RETURN_IF_FAILED(m_ipin_speed_cmd.Create("speed_cmd", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_speed_cmd));

    // create pose output pin without media description
    RETURN_IF_FAILED(m_opin_pose_buffer.Create("pose_buffer" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_opin_pose_buffer));

    ///////////////// TEST //////////////////
    RETURN_IF_FAILED(m_ipin_patches.Create("test_patches", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_patches));
  }
  else if (eStage == StageNormal)
  {
    // In this stage you would do further initialisation and/or create your dynamic pins.
    // Please take a look at the demo_dynamicpin example for further reference.
    m_steering_drift = GetPropertyInt("Steering_drift");
    m_publish_rate = GetPropertyInt("Publish_rate_ms");
    SetInterval(m_publish_rate * 1000);
  }
  else if (eStage == StageGraphReady)
  {
    // All pin connections have been established in this stage so you can query your pins
    // about their media types and additional meta data.
    // Please take a look at the demo_imageproc example for further reference.
  }

  RETURN_NOERROR;
}

tResult OdometrySimulatorFilter::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception: 
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.
    
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult OdometrySimulatorFilter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
  // first check what kind of event it is
  if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    // so we received a media sample, so this pointer better be valid.
    RETURN_IF_POINTER_NULL(pMediaSample && m_pCoderDescSignalInput != NULL);

    // by comparing it to our member pin variable we can find out which pin received
    // the sample
    if (pSource == &m_ipin_steering)
    {
      // read-out the incoming Media Sample
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);

      m_current_steering = (int8_t)value;
    }
    else if (pSource == &m_ipin_speed_cmd)
    {
      // read-out the incoming Media Sample
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);

      m_current_speed = value;

#ifdef KATANA_ODOMETRY_SIMULATOR_DEBUG
  static u_int32_t counter = 0;
  if (counter % 5 == 0)
    std::cout <<"Odometry simulator: Speed update: " <<m_current_speed <<std::endl;

  counter++;
#endif
    }
    else if (pSource == &m_ipin_patches)
    {
      assert(pMediaSample->GetSize() > 0);

      u_int8_t status;
      size_t array_size = pMediaSample->GetSize() - sizeof(u_int8_t);
      size_t num_patches = array_size/sizeof(sPatch);

      pMediaSample->CopyBufferTo(&status, sizeof(status), 0, 0);

      sPatch* sp = nullptr;
      if (num_patches > 0)
      {
        sp = new sPatch[num_patches];
        pMediaSample->CopyBufferTo(sp, array_size, sizeof(status), 0);
      }

#ifdef KATANA_ODOMETRY_SIMULATOR_PATCH_INPUT_DEBUG
      std::cout <<"OdometrySimulator: New patches" <<std::endl;
      std::cout <<"Status: " <<(u_int32_t)status <<std::endl;
      for (size_t i = 0 ; i < num_patches; i++)
      {
        std::cout <<"PATCH NUM: " <<i <<" - " <<sp[i].patch_type <<std::endl;
      }
#endif

      if (sp != nullptr)
        delete[] sp;
    }
  }

  RETURN_NOERROR;
}

float OdometrySimulatorFilter::getDistanceInInterval(float current_speed)
{
  // 0-20 = 0 Units/second
  // 100 = 60.000 Units/second = 6m/second = 21.6 km/h

  bool backward = current_speed < 0.0f;
  if (backward)
    current_speed *= -1.0f;

  float ret = (float)(current_speed)*60000.0f/80*m_publish_rate/1000;

  // additional slow down
  ret /= 2;

  // always return positive value regardless of driving direction
  return ret;
}

void OdometrySimulatorFilter::updatePose()
{
  float diff = getDistanceInInterval(m_current_speed);

  if (diff == 0.0)
    return;

  // Is the vehicle moving forward?
  m_forward = m_current_speed >= 0.0;

  /************************************************
   * Calculate the direction in which to move the current vehicle position: current theta + steering
   * The angle represents forward and backward movement (diff is always positive)
   ***********************************************/

  // steering, notice driving direction
  double angle = getAngleFromSteering(m_current_steering + m_steering_drift);

  angle = diff/WHEELBASE * std::tan(angle);    // Einspurmodell, yaw diff of vehicle pose when driving distance diff

  if (!m_forward)
    angle *= -1;

  // move pose in direction <dir>
  float dir = m_forward ? m_pose.getTheta() + angle/2 : m_pose.getReverseDirection().getTheta() + angle/2;

  m_pose.x() += (_position_type)(diff * cos(dir));
  m_pose.y() += (_position_type)(diff * sin(dir));

  // How much did the vehicle turn? -> update pose
  m_pose.setTheta(m_pose.getTheta() + angle);
}

tResult OdometrySimulatorFilter::sendPose(const katana::Pose& p)
{
#ifdef KATANA_ODOMETRY_SIMULATOR_DEBUG
  static u_int32_t counter = 0;
  if (counter % 50 == 0)
    std::cout <<"Pose: " <<p.getX() <<" " <<p.getY() <<" " <<p.getTheta() <<std::endl;

  counter++;
#endif

  // fast without MediaSampleSerializer
  sPose sp;
  sp.x = p.getX();
  sp.y = p.getY();
  sp.theta = p.getTheta();
  cObjectPtr<IMediaSample> media_sample_buf;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&media_sample_buf));
  RETURN_IF_FAILED(media_sample_buf->AllocBuffer(sizeof(sPose)));
  RETURN_IF_FAILED(media_sample_buf->CopyBufferFrom(&sp, sizeof(sPose), 0, 0));
  media_sample_buf->SetTime(_clock->GetStreamTime());

  m_opin_pose_buffer.Transmit(media_sample_buf);

  RETURN_NOERROR;
}

double OdometrySimulatorFilter::getAngleFromSteering(double steering) const
{
  return steering*STEERING_SENSOR_TO_ANGLE;
}

tResult OdometrySimulatorFilter::Cycle(__exception)
{
  updatePose();
  return sendPose(m_pose);
}
