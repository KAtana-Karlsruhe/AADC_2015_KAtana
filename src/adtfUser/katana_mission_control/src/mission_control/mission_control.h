// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-04
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_H_
#define _MISSION_CONTROL_H_

#include <array>

#include "katanaCommon/katanaCommon.h"
#include "PinConversions.h"
#include "RoadSign.h"

#include "mission_control/StateManager.h"
#include "mission_control/System.h"

#include "mission_control/DrivingStripChanger.h"

#include "mission_control/states/main_states/MainStateBase.h"
#include "mission_control/states/main_states/StateDriving.h"
#include "mission_control/states/main_states/StateError.h"
#include "mission_control/states/main_states/StateRecovering.h"
#include "mission_control/states/main_states/StateWaiting.h"

namespace katana
{

class MissionControl
{

public:

  enum JURY_STATE : uint32_t {
    INIT,
    INITIALIZED,
    READY_RECEIVED,
    RUN_RECEIVED,
    STOP_RECEIVED,
    COMLETE_SEND
  };

  //! Pointer shorthand
  typedef std::shared_ptr<MissionControl> Ptr;

  //! Contructor
  MissionControl();

  //! Destructor
  virtual ~MissionControl()    {}

  //! Call when starting automated driving programm - reset and initialize sub-modules
  bool initialize(const std::string& maneuver_config,
                  const string& patch_config,
                  const std::string& mc_config,
                  FuncSteering call_output_steering,
                  FuncTorque call_output_speed,
                  TransmitPatchesFunc call_output_patches,
                  TransmitStatusFunc call_output_status,
                  TransmitDriverStateFunc call_output_driver_state,
                  LightStateFunc call_output_lights
                  );

  //! Call when graph ready
  void setReady();

  //! Cron method
  void cron(_clock_time current_streamTime);

  //! Return current state
  MainState getState() const    { return m_main_state->getState(); }

  //! Transmit patches to perception
  void transmitPatches(const std::vector<katana::RoadBase::ConstPtr>& patches, u_int8_t status, u_int8_t number_of_stitches, u_int8_t patches_to_search, double matching_threshold);

  /** ************** CONTROL ***************/
  //! Jury Command with maneuver entry
  void juryCommand(int8_t action, Maneuver_Entry maneuver_entry);

  //! Get current driver state (-> Jury driver state)
  bool getManeuver(Maneuver_Entry& maneuver_entry);



  /** ************** ACCESS ****************/
  const System::Ptr& getSystem()             { return m_system; }

  const MainStateBase::Ptr& getStateObject() { return m_main_state->getStateObject(); }

  /** ************** UPDATE ****************/
  void poseChanged(const PoseWithTime &new_pose);
  void newPatches(PatchVectorPtr patch_vector, PerceptionState perception_state);
  void newObstacles(sObstacle* obstacles, size_t number);
  void newTrafficSigns(sRoadSign* roadsigns, std::size_t number);

  //! Reset all important states, e.g. when jury stop or request ready
  void resetCarState();

  //! Handle debugTrigger
  void debugTrigger(u_int8_t code);


private:

  //! MainState of mission control
  StateManager<MainStateBase, MainState, MainState::STATE_COUNT>::Ptr m_main_state;

  //! Submodules
  System::Ptr m_system;

  //!
  DrivingStripChanger::Ptr m_driving_strip_changer;

  //! Filter function for transmitting patches
  TransmitPatchesFunc m_transmit_patches_func;

  //! current pose
  PoseWithTime m_current_pose;

  //! Last stream time
  _clock_time m_last_stream_time;

  //! state of the jury module
  JURY_STATE m_jury_state;

};


} //ns


#endif // _MISSION_CONTROL_H_
