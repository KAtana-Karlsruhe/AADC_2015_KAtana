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
#include "odometry_filter.h"


#ifdef KATANA_ODOMETRY_DEBUG
#include <iostream>

void outputPose(const Pose& p)
{
  std::cout <<"X: " <<p.getX() <<" Y: " <<p.getY() <<" Theta: " <<p.getTheta() <<std::endl;
}
#endif
#ifdef KATANA_REVERSE_DRIVING_DEBUG
#include <iostream>
#endif
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
#include <iostream>
#endif

/// Create filter shell
ADTF_FILTER_PLUGIN("AADC Odometry Filter", OID_ADTF_ODOMETRY_FILTER, OdometryFilter);


OdometryFilter::OdometryFilter(const tChar* __info)
  : cTimeTriggeredFilter(__info)
  , m_target_speed(0)
  , m_forward(true)
  , m_ticks_left(0)
  , m_ticks_right(0)
  , m_current_ticks_left(0)
  , m_current_ticks_right(0)
  , m_odometry_error_counter(0)
{
  //LOG_INFO(adtf_util::cString::Format("counter %d...%d , rpm = %f ",counterValue,lastCounterValue,rpm));
  m_steering = new SignalAverage<float, u_int32_t>(400);

  // Gnuplot
#ifdef KATANA_ODOMETRY_DEBUB_PLOT
  m_gnuplot_file.open("/tmp/odometry_gnuplot.gpldata");
  m_gnuplot_file <<"#Plot this file with: plot <filename> using 1:2 with lp" <<std::endl
                 <<"#set size ratio -1" <<std::endl;
#endif
}

OdometryFilter::~OdometryFilter()
{
  if (m_steering != nullptr)
    delete m_steering;

#ifdef KATANA_ODOMETRY_DEBUB_PLOT
  m_gnuplot_file.close();
#endif
}
/**
 *   The Filter Init Function.
 *    eInitStage ... StageFirst ... should be used for creating and registering Pins
 *               ... StageNormal .. should be used for reading the properies and initalizing
 *                                  everything before pin connections are made
 *   see {@link IFilter#Init IFilter::Init}.
 *
 */
tResult OdometryFilter::Init(tInitStage eStage, __exception)
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
    RETURN_IF_FAILED(m_ipin_wheel_left.Create("wheel_left", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_wheel_left));
    RETURN_IF_FAILED(m_ipin_wheel_right.Create("wheel_right", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_wheel_right));
    RETURN_IF_FAILED(m_ipin_steering.Create("steering_angle", pTypeSignalValue, this));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_steering));


    // status input pin without media description
    RETURN_IF_FAILED(m_ipin_status.Create("odo_status" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_ipin_status));


    // create pose output pin without media description
    RETURN_IF_FAILED(m_opin_pose.Create("pose_buffer" , new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), NULL));
    RETURN_IF_FAILED(RegisterPin(&m_opin_pose));


    m_steering->clear();
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

tResult OdometryFilter::Shutdown(tInitStage eStage, __exception)
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

tResult OdometryFilter::OnPinEvent(IPin* pSource,
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
    if (pSource == &m_ipin_wheel_left)
    {
      // remove first ticks from arduino
      /*if (remove_first_ticks)
      {
        --remove_first_ticks;
        RETURN_NOERROR;
      }*/

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

      m_current_ticks_left = (u_int64_t)value;

      updatePose();
    }
    else if (pSource == &m_ipin_wheel_right)
    {
      // remove first ticks from arduino
      /*if (remove_first_ticks)
      {
        --remove_first_ticks;
        RETURN_NOERROR;
      }*/

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

      m_current_ticks_right = (u_int64_t)value;

      updatePose();
    }
    else if (pSource == &m_ipin_steering)
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

      m_steering->addSample(SignalAverage<float, u_int32_t>::Sample(value, timeStamp));

    }
    else if (pSource == &m_ipin_status)
    {    
      assert(sizeof(katana::sStatus) == pMediaSample->GetSize() && "Odometry: MediaSample has not same size as sizeof(katana::sStatus)");

      katana::sStatus status;
      pMediaSample->CopyBufferTo(&status, sizeof(katana::sStatus), 0, 0);

      // save in flag (set on forward whenenver status is not REVERSE)
      if (status.status == katana::MissionControlStatus::DRIVING_FORWARD || status.status == katana::MissionControlStatus::DRIVING_REVERSE)
      {
#ifdef KATANA_REVERSE_DRIVING_DEBUG
        std::cout <<"[Odometry] Switching to " <<(status.status == katana::MissionControlStatus::DRIVING_FORWARD ? "forward" : "reverse") <<" mode" <<std::endl;
#endif

        m_forward = status.status != katana::MissionControlStatus::DRIVING_REVERSE;
      }

    }
  }

  RETURN_NOERROR;
}

void OdometryFilter::updatePose()
{
  // we are in waiting state
  if (m_ignore_counter)
    return;

  // lock this function
  std::lock_guard<std::mutex> lock(m_pose_update_mutex);

  // this is bad: arduino sends bad data, maybe has restarted
  if (m_current_ticks_left < m_ticks_left || m_current_ticks_right < m_ticks_right)
  {
    // Reset odometry internal tick counter (but not the Pose!)
    m_ticks_left = 0;
    m_ticks_right = 0;
    m_odometry_error_counter = 0;
    return;
  }

#ifdef KATANA_ODOMETRY_DEBUG
  // check if odometry is wrong
  int32_t max_ticks = std::max(m_current_ticks_right, m_current_ticks_left);
  int32_t min_ticks = std::min(m_current_ticks_right, m_current_ticks_left);
  if ((max_ticks > 20 || min_ticks > 20) && (float)(max_ticks+1)/(min_ticks+1) > 2.0)
  {
    LOG_INFO(adtf_util::cString::Format("Wrong odometry! (left/right) %d/%d",m_current_ticks_left,m_current_ticks_right));
  }
#endif

  const int32_t diff_left = m_current_ticks_left - m_ticks_left;
  const int32_t diff_right = m_current_ticks_right - m_ticks_right;

  if (!m_odometry_error && diff_left != 0)
  {
    m_odometry_error_counter = 0;
    m_odometry_error = true;
  }
  else if (m_odometry_error && diff_right != 0)
  {
    m_odometry_error_counter = 0;
    m_odometry_error = false;
  }
  else
  {
    if ((m_odometry_error && diff_left != 0) || (!m_odometry_error && diff_right != 0))
    {
      ++m_odometry_error_counter;
    }
  }

  if (m_odometry_error_counter > 2)
  {
    if (m_odometry_error_counter == 3)
    {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout <<"[Odometry] Odometry ERROR - Sensor " <<(m_odometry_error_counter ? "right" : "left") <<"! Skipping updating pose" <<std::endl;
#endif
    }

    m_ticks_left = m_current_ticks_left;
    m_ticks_right = m_current_ticks_right;

    return;
  }

  float diff = 0.0f;

  diff += diff_left;
  diff += diff_right;

  if (diff == 0.0)
    return;

  // Average of left and right wheel distance
  diff /= 2;
  diff *= DISTANCE_PER_TICK;

  // Save distances as new distances
  m_ticks_left = m_current_ticks_left;
  m_ticks_right = m_current_ticks_right;

  /************************************************
   * Calculate the direction in which to move the current vehicle position: current theta + steering
   * The angle represents forward and backward movement (diff is always positive)
   ***********************************************/

  // steering, notice driving direction
  double angle = getAngleFromSteering(m_steering->get());

  angle = diff/WHEELBASE * std::tan(angle);    // Einspurmodell, yaw diff of vehicle pose when driving distance diff

  if (!m_forward)
    angle *= -1;

  // move pose in direction <dir>
  float dir = m_forward ? m_pose.getTheta() + angle/2 : m_pose.getReverseDirection().getTheta() + angle/2;

  m_pose.x() += (_position_type)(diff * cos(dir));
  m_pose.y() += (_position_type)(diff * sin(dir));

#ifdef KATANA_ODOMETRY_DEBUG
  std::cout <<"Angle:   " <<angle <<std::endl;
#endif

  // How much did the vehicle turn? -> update pose
  m_pose.setTheta(m_pose.getTheta() + angle);

#ifdef KATANA_ODOMETRY_DEBUB_PLOT
  m_gnuplot_file <<m_pose.x() <<"   " <<m_pose.y() <<"  " <<m_pose.getTheta() <<std::endl;
#endif
}

tResult OdometryFilter::sendPose(const katana::Pose& p)
{
  if (m_ignore_counter)
  {
    --m_ignore_counter;

    // last iteration, reset stuff
    if (m_ignore_counter == 0)
    {
      m_ticks_left = m_current_ticks_left;
      m_ticks_right = m_current_ticks_right;
      m_odometry_error_counter = 0;
      m_pose = katana::Pose();
    }
    RETURN_NOERROR;
  }


#ifdef KATANA_ODOMETRY_DEBUG
  outputPose(p);
#endif

  // fast without MediaSampleSerializer
  sPose sp;
  sp.x = p.getX();
  sp.y = p.getY();
  sp.theta = p.getTheta();
  cObjectPtr<IMediaSample> media_sample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&media_sample));
  RETURN_IF_FAILED(media_sample->AllocBuffer(sizeof(sPose)));
  RETURN_IF_FAILED(media_sample->CopyBufferFrom(&sp, sizeof(sPose), 0, 0));
  media_sample->SetTime(_clock->GetStreamTime());
  m_opin_pose.Transmit(media_sample);

  RETURN_NOERROR;
}

double OdometryFilter::getAngleFromSteering(double steering) const
{
  return steering*STEERING_SENSOR_TO_ANGLE;
}

tResult OdometryFilter::Cycle(__exception)
{
  return sendPose(m_pose);
}
