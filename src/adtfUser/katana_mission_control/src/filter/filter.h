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


#ifndef _MISSION_CONTROL_FILTER_H_
#define _MISSION_CONTROL_FILTER_H_

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

#include "mission_control/mission_control.h"


using namespace adtf;

#define OID_ADTF_MISSION_CONTROL_FILTER "adtf.aadc.mission_control"
//namespace mission_control
//{

//*************************************************************************************************
class Filter : public adtf::cAsyncDataTriggeredFilter
{
  ADTF_FILTER(OID_ADTF_MISSION_CONTROL_FILTER, "KATANA Mission Control Filter", adtf::OBJCAT_DataFilter)

public:
  Filter(const tChar* __info);
  virtual ~Filter();

protected:

  //! Filter stages
  tResult Init(tInitStage eStage, __exception);
  tResult Shutdown(tInitStage eStage, __exception);

  //! implements IPinEventSink
  tResult OnAsyncPinEvent(IPin* pSource,
                     tInt nEventCode,
                     tInt nParam1,
                     tInt nParam2,
                     IMediaSample* pMediaSample);

  //! Methods to create Pins
  tResult CreatePoseInputPin(__exception);
  tResult CreatePatchInputPin(__exception);
  tResult CreateJuryPins(__exception);
  tResult CreateSignPin(__exception);
  tResult CreateCarOutputPins(__exception);       //< Speed, Steering
  tResult CreateObstacleInputPin(__exception);
  tResult CreateLightOutputPins(__exception);

  //! Initialize Mission Control in Stage Normal
  tResult initializeMissionControl();

  cObjectPtr<IMediaDescriptionManager> m_pDescManager;



  /** ********** ODOMETRY INTERFACE **********************/
  cInputPin     m_ipin_pose;

  /** ********** OBSTACLES **************/
  cInputPin     m_ipin_obstacles;

  /** ********* ROAD SIGNS **************************/
  cInputPin				m_ipin_roadSign;

  /** ********** JURY CONTROL/DRIVER STATE INTERFACE **********/
  //! input pin for the run command
  cInputPin              m_ipin_jury;
  //! output pin for state from driver
  cOutputPin             m_opin_driverstate;

  //! Coder Descriptor
  cObjectPtr<IMediaTypeDescription> m_pCoderDescJuryIn;
  //! Coder Descriptor
  cObjectPtr<IMediaTypeDescription> m_pCoderDescDriverState;
  tInt m_driverstate_size;

  /** ********** OUTPUT SPEED ******************/
  //! Output pin
  cOutputPin            m_opin_speed;
  //! Coder Descriptor output speed
  cObjectPtr<IMediaTypeDescription> m_pCoderSpeedOutput;
  tInt m_speed_size;

  /** ********** OUTPUT STEERING ******************/
  //! Output pin
  cOutputPin            m_opin_steering;
  //! Coder Descriptor output speed
  cObjectPtr<IMediaTypeDescription> m_pCoderSteeringOutput;
  tInt m_steering_size;

  /** ********** OUTPUT STATUS *****************/
  cOutputPin            m_opin_status;

  /** ********** PATCH INTERFACE **********************/
  //! Output pin
  cOutputPin            m_opin_patches;
  cInputPin		m_ipin_patch;

  /** ********** LIGHT INTERFACE **********************/
  //! Output pin
  std::array<cOutputPin, katana::NUMBER_OF_LIGHTS> m_lightPinArray;
  //! Coder Descriptor light
  cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignalOutput;
  tInt m_lights_size;

  //! Debug input pin
  cInputPin m_iDebug;
  u_int8_t i_Debug;


  /** ********** MISSION CONTROL ***********/
  //! Mission Control: the filter needs to explicitly delete the mission control in shutdown stages,
  //! so that ADTF waits for all threads to finish
  katana::MissionControl* m_mission_control;

private:
  tResult transmitSpeed(float speed);
  tResult transmitSteering(float steering);
  void transmitSpeedWrapper(float speed)     { transmitSpeed(speed); }
  void transmitSteeringWrapper(float steering)     { transmitSteering(steering); }

  tResult transmitDriverState(katana::SendAction state, katana::Maneuver_Entry maneuver_entry);
  void transmitDriverStateWrapper(katana::SendAction state, katana::Maneuver_Entry maneuver_entry) { transmitDriverState(state, maneuver_entry); }

  tResult transmitPatches(const std::vector<katana::RoadBase::ConstPtr>& patches, u_int8_t status, u_int8_t number_of_stitches, u_int8_t patches_to_search, double matching_threshold);
  void transmitPatchesWrapper(const std::vector<katana::RoadBase::ConstPtr>& patches, u_int8_t status, u_int8_t number_of_stitches, u_int8_t patches_to_search, double matching_threshold)
  {
    transmitPatches(patches, status, number_of_stitches, patches_to_search, matching_threshold);
  }

  tResult transmitStatus(katana::MissionControlStatus status);
  void transmitStatusWrapper(katana::MissionControlStatus status)                          { transmitStatus(status); }

  tResult transmitLights(const katana::LightState &lightArray);
  void transmitLightsWrapper(const katana::LightState lightArray)                          { transmitLights(lightArray); }

  //! Method running regularly onPinEvent
  void cron();
};

//*************************************************************************************************
//}   //ns

#endif // _MISSION_CONTROL_FILTER_H_
