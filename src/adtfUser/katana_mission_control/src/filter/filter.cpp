// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \author  Philipp Herweck <herweck@fzi.de>
 * \date    2014-11-29
 *
 */
//----------------------------------------------------------------------

#include "filter.h"

#include <Obstacle.h>

#include <locale>

#include <iostream>

using namespace adtf;
using namespace oadrive::core;

//namespace mission_control
//{
/// Create filter shell
ADTF_FILTER_PLUGIN("KATANA Mission Control Filter", OID_ADTF_MISSION_CONTROL_FILTER, Filter)

Filter::Filter(const tChar* __info)
  : cAsyncDataTriggeredFilter(__info)
  , m_mission_control(nullptr)
{
  // set locale (read XML numbers correctly)
  std::setlocale(LC_ALL, "C");


  //LOG_INFO(adtf_util::cString::Format("counter %d...%d , rpm = %f ",counterValue,lastCounterValue,rpm));

  // Maneuver XML
  SetPropertyStr("Maneuver-Config","");
  SetPropertyBool("Maneuver-Config" NSSUBPROP_FILENAME, tTrue);
  SetPropertyStr("Maneuver-Config" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

  // Patch XML
  SetPropertyStr("Patch-Config","");
  SetPropertyBool("Patch-Config" NSSUBPROP_FILENAME, tTrue);
  SetPropertyStr("Patch-Config" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

  // Config XML
  SetPropertyStr("MC-Config","");
  SetPropertyBool("MC-Config" NSSUBPROP_FILENAME, tTrue);
  SetPropertyStr("MC-Config" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
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
  RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr))

  // in StageFirst you can create and register your static pins.
  if (eStage == StageFirst)
  {
    // Create all needed Pins
    RETURN_IF_FAILED(CreatePoseInputPin(__exception_ptr));
    RETURN_IF_FAILED(CreatePatchInputPin(__exception_ptr));
    RETURN_IF_FAILED(CreateJuryPins(__exception_ptr));
    RETURN_IF_FAILED(CreateSignPin(__exception_ptr));
    RETURN_IF_FAILED(CreateCarOutputPins(__exception_ptr));
    RETURN_IF_FAILED(CreateObstacleInputPin(__exception_ptr));
    RETURN_IF_FAILED(CreateLightOutputPins(__exception_ptr));

    // debug pin
    RETURN_IF_FAILED(m_iDebug.Create("mc_debug_input", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), this));
    RETURN_IF_FAILED(RegisterPin(&m_iDebug));

  }
  else if (eStage == StageNormal)
  {
    // In this stage you would do further initialisation and/or create your dynamic pins.
    // Please take a look at the demo_dynamicpin example for further reference.

    return initializeMissionControl();
  }
  else if (eStage == StageGraphReady)
  {
    // All pin connections have been established in this stage so you can query your pins
    // about their media types and additional meta data.
    // Please take a look at the demo_imageproc example for further reference.

    // Calculate sizes of media samples
    cObjectPtr<IMediaSerializer> pSerializer;

    m_pCoderDescDriverState->GetMediaSampleSerializer(&pSerializer);
    m_driverstate_size = pSerializer->GetDeserializedSize();

    m_pCoderSpeedOutput->GetMediaSampleSerializer(&pSerializer);
    m_speed_size = pSerializer->GetDeserializedSize();

    m_pCoderSteeringOutput->GetMediaSampleSerializer(&pSerializer);
    m_steering_size = pSerializer->GetDeserializedSize();

    m_pCoderDescBoolSignalOutput->GetMediaSampleSerializer(&pSerializer);
    m_lights_size = pSerializer->GetDeserializedSize();

    // Let mission  control start -> set a state
    m_mission_control->setReady();

  }

  RETURN_NOERROR;
}

tResult Filter::initializeMissionControl()
{
  assert(m_mission_control == nullptr);

  m_mission_control = new katana::MissionControl();

  // Maneuver XML
  cFilename fileConfigManeuver = GetPropertyStr("Maneuver-Config");
  if (fileConfigManeuver.IsEmpty())
  {
    LOG_ERROR("Mission Control: Maneuver configuration file not found");
    RETURN_ERROR(ERR_INVALID_FILE);
  }
  ADTF_GET_CONFIG_FILENAME(fileConfigManeuver);
  fileConfigManeuver = fileConfigManeuver.CreateAbsolutePath(".");

  // Patch XML
  cFilename fileConfigPatch = GetPropertyStr("Patch-Config");
  if (fileConfigPatch.IsEmpty())
  {
    LOG_ERROR("Mission Control: Patch configuration file not found");
    RETURN_ERROR(ERR_INVALID_FILE);
  }
  ADTF_GET_CONFIG_FILENAME(fileConfigPatch);
  fileConfigPatch = fileConfigPatch.CreateAbsolutePath(".");

  // Config XML
  cFilename fileConfigMC = GetPropertyStr("MC-Config");
  if (fileConfigMC.IsEmpty())
  {
    LOG_ERROR("Mission Control: MC configuration file not found");
    RETURN_ERROR(ERR_INVALID_FILE);
  }
  ADTF_GET_CONFIG_FILENAME(fileConfigMC);
  fileConfigMC = fileConfigMC.CreateAbsolutePath(".");

  // Tell mission_controls location of configs
  if (cFileSystem::Exists(fileConfigManeuver) && cFileSystem::Exists(fileConfigPatch))
  {
    if (!m_mission_control->initialize(std::string(fileConfigManeuver),
                                 std::string(fileConfigPatch),
                                 std::string(fileConfigMC),
                                 std::bind(&Filter::transmitSteeringWrapper, this, std::placeholders::_1),
                                 std::bind(&Filter::transmitSpeedWrapper, this, std::placeholders::_1),
                                 std::bind(&Filter::transmitPatchesWrapper, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
                                 std::bind(&Filter::transmitStatusWrapper, this, std::placeholders::_1),
                                 std::bind(&Filter::transmitDriverStateWrapper, this, std::placeholders::_1, std::placeholders::_2),
                                 std::bind(&Filter::transmitLightsWrapper, this, std::placeholders::_1) ))
    {
      RETURN_ERROR(ERR_INVALID_FILE);
    }
  }
  else
  {
    RETURN_ERROR(ERR_INVALID_FILE);
  }

  RETURN_NOERROR;
}

tResult Filter::CreatePoseInputPin(__exception)
{
  // create pin
  RETURN_IF_FAILED(m_ipin_pose.Create("mc_pose_buf", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), this));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_pose));

  RETURN_NOERROR;
}

tResult Filter::CreateObstacleInputPin(__exception)
{
  // create and register the input pin
  RETURN_IF_FAILED(m_ipin_obstacles.Create("mc_obstacles", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), this));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_obstacles));

  RETURN_NOERROR;
}

tResult Filter::CreatePatchInputPin(__exception)
{
  // create and register the input pin
  RETURN_IF_FAILED(m_ipin_patch.Create("mc_patches_input", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), this));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_patch));

  RETURN_NOERROR;
}

tResult Filter::CreateJuryPins(__exception)
{
  cObjectPtr<IMediaDescriptionManager> pDescManager;
  RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

  // Jury Input
  tChar const * strDescJuryRun = pDescManager->GetMediaDescription("tJuryStruct");
  RETURN_IF_POINTER_NULL(strDescJuryRun);
  cObjectPtr<IMediaType> pTypeJuryRun = new cMediaType(0, 0, 0, "tJuryStruct", strDescJuryRun,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(pTypeJuryRun->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescJuryIn));
  RETURN_IF_FAILED(m_ipin_jury.Create("mc_Jury", new cMediaType(0, 0, 0, "tJuryStruct"), static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_jury));

  // Stop Input
  /*tChar const * strDescJuryStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");
  RETURN_IF_POINTER_NULL(strDescJuryStop);
  cObjectPtr<IMediaType> pTypeJuryStop = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescJuryStop,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(pTypeJuryStop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescStopIn));
  RETURN_IF_FAILED(m_ipin_stop.Create("mc_Jury_Stop", new cMediaType(0, 0, 0, "tJuryEmergencyStop"), static_cast<IPinEventSink*> (this)));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_stop));*/

  // Driver State Output
  tChar const * strDescDriverState = pDescManager->GetMediaDescription("tDriverStruct");
  RETURN_IF_POINTER_NULL(strDescDriverState);
  cObjectPtr<IMediaType> pTypeDriverState = new cMediaType(0, 0, 0, "tDriverStruct", strDescDriverState, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(m_opin_driverstate.Create("mc_Jury_Driver_State", pTypeDriverState, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_opin_driverstate));
  RETURN_IF_FAILED(pTypeDriverState->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverState));

  RETURN_NOERROR;
}

tResult Filter::CreateSignPin(__exception)
{
  // create and register the input pin
  RETURN_IF_FAILED(m_ipin_roadSign.Create("mc_traffic_signs_input", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), this));
  RETURN_IF_FAILED(RegisterPin(&m_ipin_roadSign));

  RETURN_NOERROR;
}

tResult Filter::CreateCarOutputPins(__exception)
{
  cObjectPtr<IMediaDescriptionManager> pDescManager;
  RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

  tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
  RETURN_IF_POINTER_NULL(strDescSignalValue);
  cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

  //************** SPEED **************/
  RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderSpeedOutput));

  RETURN_IF_FAILED(m_opin_speed.Create("mc_speed", pTypeSignalValue, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_opin_speed));

  //************** STEERING************/
  RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderSteeringOutput));

  RETURN_IF_FAILED(m_opin_steering.Create("mc_steering", pTypeSignalValue, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_opin_steering));

  //************* PATCHES *************/
  RETURN_IF_FAILED(m_opin_patches.Create("mc_patches", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), NULL));
  RETURN_IF_FAILED(RegisterPin(&m_opin_patches));

  //************* STATUS *************/
  RETURN_IF_FAILED(m_opin_status.Create("mc_status", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), NULL));
  RETURN_IF_FAILED(RegisterPin(&m_opin_status));

  RETURN_NOERROR;
}

tResult Filter::CreateLightOutputPins(__exception)
{
  cObjectPtr<IMediaDescriptionManager> pDescManager;
  RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

  tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
  RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
  cObjectPtr<IMediaType> pTypeBoolSignalValueOutput = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
  RETURN_IF_FAILED(pTypeBoolSignalValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolSignalOutput));

  // Headlight Output
  RETURN_IF_FAILED(m_lightPinArray[katana::LightName::HEAD_LIGHT].Create("headLightEnabled", pTypeBoolSignalValueOutput, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_lightPinArray[katana::LightName::HEAD_LIGHT]));

  // Brakelight Output
  RETURN_IF_FAILED(m_lightPinArray[katana::LightName::BREAK_LIGHT].Create("brakeLightEnabled", pTypeBoolSignalValueOutput, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_lightPinArray[katana::LightName::BREAK_LIGHT]));

  // Turn signal Output
  RETURN_IF_FAILED(m_lightPinArray[katana::LightName::TURN_LEFT].Create("turnSignalLeftEnabled", pTypeBoolSignalValueOutput, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_lightPinArray[katana::LightName::TURN_LEFT]));
  RETURN_IF_FAILED(m_lightPinArray[katana::LightName::TURN_RIGHT].Create("turnSignalRightEnabled", pTypeBoolSignalValueOutput, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_lightPinArray[katana::LightName::TURN_RIGHT]));

  // Reverselight Output
  RETURN_IF_FAILED(m_lightPinArray[katana::LightName::REVERSE_LIGHT].Create("reverseLightEnabled", pTypeBoolSignalValueOutput, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_lightPinArray[katana::LightName::REVERSE_LIGHT]));

  // Warning lights Output
  RETURN_IF_FAILED(m_lightPinArray[katana::LightName::WARNING_LIGHTS].Create("warningLightsEnabled", pTypeBoolSignalValueOutput, NULL));
  RETURN_IF_FAILED(RegisterPin(&m_lightPinArray[katana::LightName::WARNING_LIGHTS]));

  RETURN_NOERROR;
}

tResult Filter::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception: -------
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
      if (m_mission_control != nullptr)
      {
        delete m_mission_control;
        m_mission_control = nullptr;
      }
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult Filter::OnAsyncPinEvent(IPin* pSource,
                                           tInt nEventCode,
                                           tInt nParam1,
                                           tInt nParam2,
                                           IMediaSample* pMediaSample)
{
  // Run method regularly
  cron();

  // first check what kind of event it is
  if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
  {
    // so we received a media sample, so this pointer better be valid.
    RETURN_IF_POINTER_NULL(pMediaSample);

    // by comparing it to our member pin variable we can find out which pin received
    // the sample
    if (pSource == &m_ipin_pose)
    {
#ifndef NDEBUG
      assert(pMediaSample->GetSize() == sizeof(katana::sPose));
#else
        if (pMediaSample->GetSize() != sizeof(katana::sPose))
          RETURN_NOERROR;
#endif

      katana::sPose pose;
      pMediaSample->CopyBufferTo(&pose, sizeof(katana::sPose), 0, 0);

      // Create stamped pose (scaled from 0.1 mm to m
      katana::PoseWithTime pose_stamped;
      PoseTraits<Pose2d>::fromPositionAndOrientationRPY(pose_stamped.pose, pose.x * katana::COORDINATE_SCALE_FACTOR_TO_M, pose.y * katana::COORDINATE_SCALE_FACTOR_TO_M, pose.theta);
      pose_stamped.time = pMediaSample->GetTime();

      // delegate to mission control
      m_mission_control->poseChanged(pose_stamped);
    }
    else if (pSource == &m_ipin_jury && m_pCoderDescJuryIn != NULL)
    {
      cObjectPtr<IMediaCoder> pCoder;
      RETURN_IF_FAILED(m_pCoderDescJuryIn->Lock(pMediaSample, &pCoder));

      tInt16 entry = 0;
      tInt8 action = 0;

      pCoder->Get("i8ActionID", (tVoid*)&action);
      pCoder->Get("i16ManeuverEntry", (tVoid*)&entry);
      m_pCoderDescJuryIn->Unlock(pCoder);

      #ifdef KATANA_MC_JURY_DEBUG
		std::cout << "Mission control filter: received jury struct. ID: " << (u_int32_t)action << " maneuver: " << entry << std::endl;
      #endif

      m_mission_control->juryCommand(action, entry);
    }
    else if (pSource == &m_ipin_patch)
    {
      assert(pMediaSample->GetSize() > 0 && "pMediaSample->GetSize() > 0");

      // message starts with perception state
      katana::PerceptionState perception_state;
      pMediaSample->CopyBufferTo(&perception_state, sizeof(katana::PerceptionState), 0, 0);

      std::size_t n = 0;
      katana::PatchVectorPtr patch_vector = std::make_shared<katana::PatchVector>();

      const std::size_t size_patches = pMediaSample->GetSize()-sizeof(katana::PerceptionState);
      if (size_patches > 0)
      {
        n = size_patches/sizeof(katana::sPatch);

        patch_vector->resize(n);

        pMediaSample->CopyBufferTo(patch_vector->data(), size_patches, sizeof(u_int8_t), 0);

        #ifdef KATANA_MC_FILTER_DEBUG
        std::cout <<"Mission control filter: Received Patches:" <<std::endl;
        for (size_t i = 0; i < n; i++)
        {
          std::cout <<"Type: " << (u_int32_t)(*patch_vector)[i].patch_type << " ID: " <<(*patch_vector)[i].id <<std::endl;
          std::cout <<"pose: " << (*patch_vector)[i].sp.toPose2d() <<std::endl;
        }
        #endif
      }
      // delegate new patches to mission control
      m_mission_control->newPatches(patch_vector, perception_state);

    }
    else if (pSource == &m_ipin_obstacles)
    {
            // read data in sObstacle struct
      size_t n = pMediaSample->GetSize() / sizeof(katana::sObstacle);
      katana::sObstacle* obstacles = new katana::sObstacle[n];
      pMediaSample->CopyBufferTo(obstacles, pMediaSample->GetSize(), 0, 0);

      // delegate new obstacles to mission control
      m_mission_control->newObstacles(obstacles, n);

      #ifdef KATANA_MC_FILTER_DEBUG
      std::cout <<"Mission control filter: Received Obstacles:" <<std::endl;
      for (size_t i = 0; i < n; i++)
      {
        std::cout <<"Source: " << obstacles[i].source << std::endl;
        std::cout << obstacles[i].sp.toPose2d() << std::endl;
      }
      #endif

	  if(obstacles)
	    delete[] obstacles;
    }
    else if (pSource == &m_ipin_roadSign) {
      std::size_t n = pMediaSample->GetSize()/sizeof(katana::sRoadSign);
      katana::sRoadSign* roadsigns = new katana::sRoadSign[n];
      pMediaSample->CopyBufferTo(roadsigns, pMediaSample->GetSize(), 0, 0);

      #ifdef KATANA_MC_FILTER_DEBUG
      std::cout <<"Mission control filter: Received Roadsigns:" <<std::endl;
      for (size_t i = 0; i < n; i++)
      {
        std::cout <<"Type: " << (u_int32_t)roadsigns[i].sign << " Size: " <<roadsigns[i].size <<std::endl;
        std::cout <<"pose: " <<roadsigns[i].sp.toPose2d() <<std::endl;
      }
      #endif

      m_mission_control->newTrafficSigns(roadsigns, n);

	  if(roadsigns)
		delete[] roadsigns;
    }
    else if(pSource == &m_iDebug && nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // copy first byte to obtain debug code
        u_int8_t code;
        pMediaSample->CopyBufferTo(&code, sizeof(u_int8_t), 0, 0);


        if (code == 3)
        {
          std::cout <<"EXTERN TRIGGERED: Request left." <<std::endl;
          m_mission_control->getSystem()->getLightController()->setLight(katana::TURN_LEFT, true);
        }
        else if (code == 4)
        {
          std::cout <<"EXTERN TRIGGERED: Request right." <<std::endl;
          m_mission_control->getSystem()->getLightController()->setLight(katana::TURN_RIGHT, true);
        }
        else if (code == 5)
        {
          std::cout <<"EXTERN TRIGGERED: Request stop blinker." <<std::endl;
          m_mission_control->getSystem()->getLightController()->setLight(katana::TURN_LEFT, false);
          m_mission_control->getSystem()->getLightController()->setLight(katana::TURN_RIGHT, false);
        }
        else {
	  // let mc handle trigger
	  m_mission_control->debugTrigger(code);
	}
    }
  }

  RETURN_NOERROR;
}

void Filter::cron()
{
  m_mission_control->cron(_clock->GetStreamTime());
}

tResult Filter::transmitDriverState(katana::SendAction action, katana::Maneuver_Entry maneuver_entry)
{
  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));


  RETURN_IF_FAILED(pMediaSample->AllocBuffer(m_driverstate_size));

  cObjectPtr<IMediaCoder> pCoder;
  RETURN_IF_FAILED(m_pCoderDescDriverState->WriteLock(pMediaSample, &pCoder));

  #ifdef KATANA_MC_JURY_DEBUG
    std::cout << "Mission control filter transmitting driver state. Action: " << (int32_t)action << " entry: " << maneuver_entry << std::endl;
  #endif
  int16_t entry = (int16_t) maneuver_entry;
  int8_t state = (int8_t) action;
  pCoder->Set("i8StateID", (tVoid*)&state);
  pCoder->Set("i16ManeuverEntry", (tVoid*)&entry);
  m_pCoderDescDriverState->Unlock(pCoder);

  pMediaSample->SetTime(_clock->GetStreamTime());
  m_opin_driverstate.Transmit(pMediaSample);

  RETURN_NOERROR;
}

tResult Filter::transmitSpeed(float speed)
{
  //values to transmit
  tUInt32 timeStamp = 0;

  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

  RETURN_IF_FAILED(pMediaSample->AllocBuffer(m_speed_size));

  cObjectPtr<IMediaCoder> pCoder;
  RETURN_IF_FAILED(m_pCoderSpeedOutput->WriteLock(pMediaSample, &pCoder));

  pCoder->Set("f32Value", (tVoid*)&speed);
  pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
  m_pCoderSpeedOutput->Unlock(pCoder);

  pMediaSample->SetTime(_clock->GetStreamTime());
  m_opin_speed.Transmit(pMediaSample);

  RETURN_NOERROR;
}

tResult Filter::transmitSteering(float steering)
{
  //values to transmit
  tUInt32 timeStamp = 0;

#ifdef KATANA_MC_FILTER_TRANSMIT_DEBUG
  std::cout <<"Filter [transmitSteering] - Value: " << steering <<std::endl;
#endif

  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

  RETURN_IF_FAILED(pMediaSample->AllocBuffer(m_steering_size));

  cObjectPtr<IMediaCoder> pCoder;
  RETURN_IF_FAILED(m_pCoderSteeringOutput->WriteLock(pMediaSample, &pCoder));

  pCoder->Set("f32Value", (tVoid*)&steering);
  pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
  m_pCoderSteeringOutput->Unlock(pCoder);

  pMediaSample->SetTime(_clock->GetStreamTime());
  m_opin_steering.Transmit(pMediaSample);

  RETURN_NOERROR;
}

tResult Filter::transmitPatches(const std::vector<katana::RoadBase::ConstPtr>& patches, u_int8_t status, u_int8_t number_of_stitches, u_int8_t patches_to_search,
                                double matching_threshold)
{
#ifdef KATANA_MC_FILTER_TRANSMIT_DEBUG
  std::cout <<"Filter [transmitPatches]" <<std::endl;
#endif

  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

  static const std::size_t s_status = sizeof(status);
  static const std::size_t s_number = sizeof(number_of_stitches);
  static const std::size_t s_patches = sizeof(patches_to_search);
  static const std::size_t s_threshold = sizeof(matching_threshold);

  const std::size_t size_patches = sizeof(katana::sPatch) * patches.size();

  const std::size_t s_complete = s_status + s_number + s_patches + size_patches + s_threshold;

  RETURN_IF_FAILED(pMediaSample->AllocBuffer(s_complete));

  // write status byte
  pMediaSample->CopyBufferFrom(&status, s_status, 0, 0);
  // write number stitches
  pMediaSample->CopyBufferFrom(&number_of_stitches, s_number, 0, s_status);
  // write patches to search
  pMediaSample->CopyBufferFrom(&patches_to_search, s_patches, 0, s_status + s_number);
  // write patches to search
  pMediaSample->CopyBufferFrom(&matching_threshold, s_threshold, 0, s_status + s_number + s_patches);

  // write patches
  // create patch array
  katana::sPatch* sp = new katana::sPatch[patches.size()];

  #ifdef KATANA_MC_FILTER_DEBUG
    if(patches.size() > 0) {
      std::cout << "MC Filter transmitting to lanetracker: ";
    }
  #endif

  for (std::size_t i = 0; i < patches.size(); i++)
  {
    sp[i].sp.fromPoseScaled(patches[i]->getAnchorPose());
    sp[i].patch_type = (int32_t)patches[i]->getPatchType();
    sp[i].id = patches[i]->getId();

    #ifdef KATANA_MC_FILTER_DEBUG
      std::cout << patches[i]->getAnchorPose() << " type: " << sp[i].patch_type << " id: " << sp[i].id << std::endl;
    #endif
  }

  // copy patch array
  pMediaSample->CopyBufferFrom(sp, size_patches, 0, s_status + s_number + s_patches + s_threshold);

  // set stream time
  pMediaSample->SetTime(_clock->GetStreamTime());

  // delete array
  delete[] sp;

  // send media sample
  m_opin_patches.Transmit(pMediaSample);

  RETURN_NOERROR;
}
tResult Filter::transmitStatus(katana::MissionControlStatus status)
{
  katana::sStatus s;
  s.status = status;

  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
  RETURN_IF_FAILED(pMediaSample->AllocBuffer(sizeof(katana::sStatus)));
  pMediaSample->CopyBufferFrom(&s, sizeof(katana::sStatus), 0, 0);
  pMediaSample->SetTime(_clock->GetStreamTime());

  m_opin_status.Transmit(pMediaSample);

  RETURN_NOERROR;
}

tResult Filter::transmitLights(const katana::LightState& lightArray)
{
  //write values with zero
  tUInt32 timeStamp = 0;
  tBool curr = false;
  for (size_t i = 0; i < katana::NUMBER_OF_LIGHTS; ++i)
  {
    // skip don't care flags
    if (lightArray[i] == 2)
      continue;

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(m_lights_size));

    cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescBoolSignalOutput->WriteLock(pMediaSample, &pCoder));

    curr = (lightArray[i] == 1) ? true : false;

#ifdef KATANA_MC_FILTER_TRANSMIT_DEBUG
    std::cout <<"Filter [transmitLights] - LightNumber: " << i << " Value: " << curr <<std::endl;
#endif

    pCoder->Set("bValue", (tVoid*)&curr);
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescBoolSignalOutput->Unlock(pCoder);

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_lightPinArray[i].Transmit(pMediaSample);

  }
  RETURN_NOERROR;
}


//} //ns
