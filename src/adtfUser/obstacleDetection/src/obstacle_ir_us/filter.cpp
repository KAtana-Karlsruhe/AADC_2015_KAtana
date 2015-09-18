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
#include "filter.h"

//#define DEBUG_OUTPUT

#include <Obstacle.h>

#ifdef KATANA_IR_US_OBSTACLES_DEBUG
#include <iostream>
#endif

/// Create filter shell
ADTF_FILTER_PLUGIN("KATANA Obstacle IR/US Filter", OID_ADTF_OBSTACLE_FILTER, Filter)

Filter::Filter(const tChar* __info)
  : cTimeTriggeredFilter(__info)
{
  //LOG_INFO(adtf_util::cString::Format("counter %d...%d , rpm = %f ",counterValue,lastCounterValue,rpm));

//  for (SignalVariance& i : m_signals)
//  {
//    i.setNumber(SAMPLE_NUMBER);
//  }

  for (size_t i =0; i< SENSOR_COUNT; ++i)
  {
    if(i < ObstacleSource::US_FRONT_LEFT)
    {
      m_signals[i].setNumber(SAMPLE_NUMBER_IR);
    }else{
      m_signals[i].setNumber(SAMPLE_NUMBER_US);
    }
//    SignalVariance& i : m_signals
  }


  m_sensor_data[IR_FRONT_CENTER_LONG].MAX = IR_LONG_MAX;
  m_sensor_data[IR_FRONT_CENTER_LONG].MIN = IR_LONG_MIN;
  m_sensor_data[IR_FRONT_CENTER_LONG].VARIANCE = VARIANCE_IR_LONG;
  m_sensor_data[IR_FRONT_CENTER_LONG].POSE = katana::Pose(4700, 0, 0);

  m_sensor_data[IR_FRONT_CENTER_SHORT].MAX = IR_SHORT_MAX;
  m_sensor_data[IR_FRONT_CENTER_SHORT].MIN = IR_SHORT_MIN;
  m_sensor_data[IR_FRONT_CENTER_SHORT].VARIANCE = VARIANCE_IR_SHORT;
  m_sensor_data[IR_FRONT_CENTER_SHORT].POSE = katana::Pose(4700, 0, 0);

  m_sensor_data[US_FRONT_LEFT].MAX = US_MAX;
  m_sensor_data[US_FRONT_LEFT].MIN = US_MIN;
  m_sensor_data[US_FRONT_LEFT].VARIANCE = VARIANCE_US;
  m_sensor_data[US_FRONT_LEFT].POSE = katana::Pose(4700, 650, 27.5*M_PI/180);

  m_sensor_data[US_FRONT_RIGHT].MAX = US_MAX;
  m_sensor_data[US_FRONT_RIGHT].MIN = US_MIN;
  m_sensor_data[US_FRONT_RIGHT].VARIANCE = VARIANCE_US;
  m_sensor_data[US_FRONT_RIGHT].POSE = katana::Pose(4700, -650, -27.5*M_PI/180);

  m_sensor_data[IR_FRONT_LEFT_LONG].MAX = IR_LONG_MAX;
  m_sensor_data[IR_FRONT_LEFT_LONG].MIN = IR_LONG_MIN;
  m_sensor_data[IR_FRONT_LEFT_LONG].VARIANCE = VARIANCE_IR_LONG;
  m_sensor_data[IR_FRONT_LEFT_LONG].POSE = katana::Pose(4700, 1300, 90*M_PI/180);

  m_sensor_data[IR_FRONT_LEFT_SHORT].MAX = IR_SHORT_MAX;
  m_sensor_data[IR_FRONT_LEFT_SHORT].MIN = IR_SHORT_MIN;
  m_sensor_data[IR_FRONT_LEFT_SHORT].VARIANCE = VARIANCE_IR_SHORT;
  m_sensor_data[IR_FRONT_LEFT_SHORT].POSE = katana::Pose(4700, 1300, 90*M_PI/180);

  m_sensor_data[IR_FRONT_RIGHT_LONG].MAX = IR_LONG_MAX;
  m_sensor_data[IR_FRONT_RIGHT_LONG].MIN = IR_LONG_MIN;
  m_sensor_data[IR_FRONT_RIGHT_LONG].VARIANCE = VARIANCE_IR_LONG;
  m_sensor_data[IR_FRONT_RIGHT_LONG].POSE = katana::Pose(4700, -1300, -90*M_PI/180);

  m_sensor_data[IR_FRONT_RIGHT_SHORT].MAX = IR_SHORT_MAX;
  m_sensor_data[IR_FRONT_RIGHT_SHORT].MIN = IR_SHORT_MIN;
  m_sensor_data[IR_FRONT_RIGHT_SHORT].VARIANCE = VARIANCE_IR_SHORT;
  m_sensor_data[IR_FRONT_RIGHT_SHORT].POSE = katana::Pose(4700, -1300, -90*M_PI/180);

  m_sensor_data[IR_REAR_LEFT_SHORT].MAX = IR_SHORT_MAX;
  m_sensor_data[IR_REAR_LEFT_SHORT].MIN = IR_SHORT_MIN;
  m_sensor_data[IR_REAR_LEFT_SHORT].VARIANCE = VARIANCE_IR_SHORT;
  m_sensor_data[IR_REAR_LEFT_SHORT].POSE = katana::Pose(650, 1600, 90*M_PI/180);

  m_sensor_data[IR_REAR_RIGHT_SHORT].MAX = IR_SHORT_MAX;
  m_sensor_data[IR_REAR_RIGHT_SHORT].MIN = IR_SHORT_MIN;
  m_sensor_data[IR_REAR_RIGHT_SHORT].VARIANCE = VARIANCE_IR_SHORT;
  m_sensor_data[IR_REAR_RIGHT_SHORT].POSE = katana::Pose(650, -1600, -90*M_PI/180);

  m_sensor_data[US_REAR_LEFT].MAX = US_MAX;
  m_sensor_data[US_REAR_LEFT].MIN = US_MIN;
  m_sensor_data[US_REAR_LEFT].VARIANCE = VARIANCE_US;
  m_sensor_data[US_REAR_LEFT].POSE = katana::Pose(-1200, 650, 152.5*M_PI/180);

  m_sensor_data[US_REAR_RIGHT].MAX = US_MAX;
  m_sensor_data[US_REAR_RIGHT].MIN = US_MIN;
  m_sensor_data[US_REAR_RIGHT].VARIANCE = VARIANCE_US;
  m_sensor_data[US_REAR_RIGHT].POSE = katana::Pose(-1200, -650, (207.5)*M_PI/180);

  m_sensor_data[IR_REAR_CENTER_SHORT].MAX = IR_SHORT_MAX;
  m_sensor_data[IR_REAR_CENTER_SHORT].MIN = IR_SHORT_MIN;
  m_sensor_data[IR_REAR_CENTER_SHORT].VARIANCE = VARIANCE_IR_SHORT;
  m_sensor_data[IR_REAR_CENTER_SHORT].POSE = katana::Pose(-1200, 0, 180*M_PI/180);

}

Filter::~Filter()
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
tResult Filter::Init(tInitStage eStage, __exception)
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
    RETURN_IF_FAILED(m_ipin_ir_front_center_short.Create("ir_front_center_short", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_front_center_short));
    RETURN_IF_FAILED(m_ipin_ir_front_center_long.Create("ir_front_center_long", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_front_center_long));
    RETURN_IF_FAILED(m_ipin_ir_front_left_long.Create("ir_front_left_long", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_front_left_long));
    RETURN_IF_FAILED(m_ipin_ir_front_left_short.Create("ir_front_left_short", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_front_left_short));
    RETURN_IF_FAILED(m_ipin_ir_front_right_long.Create("ir_front_right_long", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_front_right_long));
    RETURN_IF_FAILED(m_ipin_ir_front_right_short.Create("ir_front_right_short", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_front_right_short));
    RETURN_IF_FAILED(m_ipin_ir_rear_left_short.Create("ir_rear_left_short", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_rear_left_short));
    RETURN_IF_FAILED(m_ipin_ir_rear_right_short.Create("ir_rear_right_short", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_rear_right_short));
    RETURN_IF_FAILED(m_ipin_ir_rear_center_short.Create("ir_rear_center_short", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_ir_rear_center_short));
    RETURN_IF_FAILED(m_ipin_us_front_left.Create("us_front_left", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_us_front_left));
    RETURN_IF_FAILED(m_ipin_us_front_right.Create("us_front_right", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_us_front_right));
    RETURN_IF_FAILED(m_ipin_us_rear_left.Create("us_rear_left", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_us_rear_left));
    RETURN_IF_FAILED(m_ipin_us_rear_right.Create("us_rear_right", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_us_rear_right));

    // create and register the output pin
    RETURN_IF_FAILED(m_opin_obstacles.Create("obstacles" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), NULL));
    RETURN_IF_FAILED(RegisterPin(&m_opin_obstacles));

    // create and register the pose input pin
    RETURN_IF_FAILED(m_ipin_pose.Create("obstacle_pose_buf" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_pose));

  }
  else if (eStage == StageNormal)
  {
    // In this stage you would do further initialisation and/or create your dynamic pins.
    // Please take a look at the demo_dynamicpin example for further reference.
    SetInterval(PUBLISH_RATE * 1000);


  }
  else if (eStage == StageGraphReady)
  {
    // All pin connections have been established in this stage so you can query your pins
    // about their media types and additional meta data.
    // Please take a look at the demo_imageproc example for further reference.
  }

  RETURN_NOERROR;
}

tResult Filter::Shutdown(tInitStage eStage, __exception)
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

tResult Filter::OnPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
  //static u_int8_t remove_first_ticks = 10;

  // first check what kind of event it is
  if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    // so we received a media sample, so this pointer better be valid.
    RETURN_IF_POINTER_NULL(pMediaSample && m_pCoderDescSignalInput != NULL);

    // by comparing it to our member pin variable we can find out which pin received
    // the sample
    if (pSource == &m_ipin_ir_front_center_long)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);

      addSample(IR_FRONT_CENTER_LONG, value, timeStamp);
    }
    else if (pSource == &m_ipin_ir_front_center_short)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);

#ifdef KATANA_IR_US_OBSTACLES_DEBUG
  std::cout << "IR_US_Obstacles: onPinEvent IR_FRONT_SHORT value= " << value << std::endl;
#endif

      addSample(IR_FRONT_CENTER_SHORT, value, timeStamp);
    }
    else if (pSource == &m_ipin_ir_front_left_long)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(IR_FRONT_LEFT_LONG, value, timeStamp);
    }
    else if (pSource == &m_ipin_ir_front_left_short)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(IR_FRONT_LEFT_SHORT, value, timeStamp);
    }
    else if (pSource == &m_ipin_ir_front_right_long)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(IR_FRONT_RIGHT_LONG, value, timeStamp);
    }
    else if (pSource == &m_ipin_ir_front_right_short)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(IR_FRONT_RIGHT_SHORT, value, timeStamp);
    }
    else if (pSource == &m_ipin_ir_rear_left_short)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(IR_REAR_LEFT_SHORT, value, timeStamp);
    }
    else if (pSource == &m_ipin_ir_rear_right_short)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(IR_REAR_RIGHT_SHORT, value, timeStamp);
    }
    else if (pSource == &m_ipin_us_rear_right)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(US_REAR_RIGHT, value, timeStamp);
    }
    else if (pSource == &m_ipin_us_rear_left)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(US_REAR_LEFT, value, timeStamp);
    }
    else if (pSource == &m_ipin_us_front_right)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(US_FRONT_RIGHT, value, timeStamp);
    }
    else if (pSource == &m_ipin_ir_rear_center_short)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(IR_REAR_CENTER_SHORT, value, timeStamp);
    }
    else if (pSource == &m_ipin_us_front_left)
    {
      cObjectPtr<IMediaCoder> pCoderInput;
      RETURN_IF_FAILED(m_pCoderDescSignalInput->Lock(pMediaSample, &pCoderInput));

      //write values with zero
      tUInt32 timeStamp = 0;
      tFloat32 value = 0;

      //get values from media sample
      pCoderInput->Get("f32Value", (tVoid*)&value);
      pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
      m_pCoderDescSignalInput->Unlock(pCoderInput);


      addSample(US_FRONT_LEFT, value, timeStamp);
    }
    else if (pSource == &m_ipin_pose)
    {
      // update current pose
      sPose tmp;
      pMediaSample->CopyBufferTo(&tmp, sizeof(sPose), 0, 0);
      m_pose.setX(tmp.x);
      m_pose.setY(tmp.y);
      m_pose.setTheta(tmp.theta);
    }
  }

  RETURN_NOERROR;
}

void Filter::addSample(u_int8_t sensor, float value, u_int32_t stamp)
{
  assert(sensor < SENSOR_COUNT);

#ifdef KATANA_IR_US_OBSTACLES_DEBUG
//  std::cout << "IR_US_Obstacles: addSample: distance to obstacle: sensor= " << (u_int32_t) sensor <<" value= " << value << std::endl;
#endif

  m_signals[sensor].addSample(value);
}

tResult Filter::Cycle(__exception)
{

  std::map<ObstacleSource, katana::Pose> obstacle_poses;

  for (u_int8_t count = 0; count < SENSOR_COUNT; ++count)
  {
    const SignalVariance& i = m_signals[count];

    if (i.isEmpty())
      continue;

#ifdef KATANA_IR_US_OBSTACLES_DEBUG
  std::cout << "IR_US_Obstacles: IR Sensor cycle: distance to obstacle: sensortype: " << (u_int32_t) count <<" i.get()= " << i.get() << " variance = " << i.calculateVariance() << std::endl;
#endif
    if (i.get() > m_sensor_data[count].MAX)
      continue;

    if (i.get() < m_sensor_data[count].MIN)
      continue;

    if(i.calculateVariance() > m_sensor_data[count].VARIANCE)
    {
      continue;
    }

    katana::Pose vehicle_coord_pose = m_sensor_data[count].POSE.transformToWorld(katana::Pose(i.get() * 100, 0, 0));

#ifdef KATANA_IR_US_OBSTACLES_DEBUG
//  std::cout << "IR_US_Obstacles: IR Sensor cycle: distance to obstacle: i.get()= " << i.get() << " in world coordinates: x= " << vehicle_coord_pose.getX() << " , y= " << vehicle_coord_pose.getY() << std::endl;
#endif

    obstacle_poses[(ObstacleSource) count] = vehicle_coord_pose;
  }

  // Transform to world
  m_pose.transformToWorld(obstacle_poses);

  // Create sObstacleArray
  katana::sObstacle sobs[obstacle_poses.size()];

  u_int8_t i = 0;
  for(std::map<ObstacleSource, katana::Pose>::const_iterator it = obstacle_poses.begin(); it != obstacle_poses.end(); ++it)
  {
    sobs[i].sp.x = it->second.getX();
    sobs[i].sp.y = it->second.getY();
    sobs[i].sp.theta = it->second.getTheta();
    sobs[i].bounding_x = BOUNDING_X_STANDARD;
    sobs[i].bounding_y = BOUNDING_Y_STANDARD;
    sobs[i].stamp = _clock->GetStreamTime();
    sobs[i].source = it->first;
#ifdef KATANA_IR_US_OBSTACLES_DEBUG
//  if(it->first == IR_FRONT_CENTER_SHORT)
//    std::cout << "IR_US_Obstacles: IPGet IR_FRONT_SHORT position x= " << it->second.getX() << " ,y = " << it->second.getY() << std::endl;
#endif

#ifdef KATANA_IR_US_OBSTACLES_DEBUG
//  std::cout << "IR_US_Obstacles: cycle: obstacle.source: " << (u_int32_t) sobs[i].source << std::endl;
#endif
    ++i;
  }

  // Create and send MediaSample
  size_t num_bytes = obstacle_poses.size() * sizeof(sObstacle);

  cObjectPtr<IMediaSample> media_sample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&media_sample));
  RETURN_IF_FAILED(media_sample->AllocBuffer(num_bytes));
  RETURN_IF_FAILED(media_sample->CopyBufferFrom(sobs, num_bytes, 0, 0));
  media_sample->SetTime(_clock->GetStreamTime());
  m_opin_obstacles.Transmit(media_sample);

  RETURN_NOERROR;
}
