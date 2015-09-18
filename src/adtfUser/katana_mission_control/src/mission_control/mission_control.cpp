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

#include "mission_control/mission_control.h"
#include "helperClasses/HelperFunctions.h"
#include <helperClasses/WorldPlausibilityChecker.h>
#include <math.h>

namespace katana
{

MissionControl::MissionControl()
  : m_last_stream_time(0)
  , m_jury_state(INIT)
{
  // Create system container object
  m_system = std::make_shared<System>();

  // create driving strip changer
  m_driving_strip_changer.reset(new DrivingStripChanger(m_system));

  // Create MainStates
  std::array<MainStateBase::Ptr, (u_int32_t)MainState::STATE_COUNT> main_states;
  main_states[(u_int32_t)MainState::ERROR].reset(new StateError(m_system, m_driving_strip_changer));
  main_states[(u_int32_t)MainState::RECOVERING].reset(new StateRecovering(m_system, m_driving_strip_changer));
  main_states[(u_int32_t)MainState::WAITING].reset(new StateWaiting(m_system, m_driving_strip_changer));
  main_states[(u_int32_t)MainState::DRIVING].reset(new StateDriving(m_system, m_driving_strip_changer));

  m_main_state = std::make_shared<StateManager<MainStateBase, MainState, MainState::STATE_COUNT>>(main_states);
}

bool MissionControl::initialize(const std::string& maneuver_config,
                                const std::string& patch_config,
                                const std::string& mc_config,
                                FuncSteering call_output_steering,
                                FuncTorque call_output_speed,
                                TransmitPatchesFunc call_output_patches,
                                TransmitStatusFunc call_output_status,
                                TransmitDriverStateFunc call_output_driver_state,
                                LightStateFunc call_output_lights
                                )
{

  // Save filter function
  m_transmit_patches_func = call_output_patches;

  // init system and create submodules
  return m_system->Initialize(maneuver_config,
                       patch_config,
                       mc_config,
                       call_output_steering,
                       call_output_speed,
                       std::bind(&MissionControl::transmitPatches, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
                       call_output_status,
                       call_output_driver_state,
                       call_output_lights
                       );



}

void MissionControl::setReady()
{
  m_jury_state = INITIALIZED;
  m_main_state->changeState(MainState::WAITING);
}

void MissionControl::transmitPatches(const std::vector<katana::RoadBase::ConstPtr>& patches, u_int8_t status, u_int8_t number_of_stitches, u_int8_t patches_to_search,
                                     double matching_threshold) {

  if(status == (u_int8_t)PerceptionState::JUNCTION_AHEAD || status == (u_int8_t) PerceptionState::JUNCTION_AHEAD_AGAIN) {
    #ifdef KATANA_MC_PATCH_TRANSMIT_DEBUG
      std::cout << "MC: junction  AHEAD transmitting to lanetracker. Tranmitting patches:" << std::endl;
      for(RoadBase::ConstPtr segment : patches) {
	std::cout << "Type: " << (u_int32_t)segment->getPatchType() << segment->getAnchorPose() << std::endl;
      }
    #endif

    m_transmit_patches_func(patches, status, number_of_stitches, patches_to_search, matching_threshold);
  } else {
    // If junction is detected then insert straight patches for perception
    std::vector<katana::RoadBase::ConstPtr> working_copy;

    for(std::vector<katana::RoadBase::ConstPtr>::const_iterator it = patches.begin(); it != patches.end(); ++it) {

      // Do not insert junctions or parking patches
      if((*it)->getPatchType() >= PatchType::JUNCTION) {
	#ifdef KATANA_MC_PATCH_TRANSMIT_DEBUG
		std::cout << "MC: skipping junction or parking patch to transmit" << std::endl;
	#endif
      } else {
	// Copy patch to working copy
	working_copy.push_back(*it);
      }
    }
    m_transmit_patches_func(working_copy, status, number_of_stitches, patches_to_search , matching_threshold);
    //m_transmit_patches_func(patches, status, 0, 0, -1.0);
  }
}

void MissionControl::juryCommand(int8_t action_number, int16_t maneuver_entry)
{
  // return if objects not fully initialized
  if(m_jury_state == INIT) return;

  JuryAction action = HelperFunctions::getJuryAction(action_number);

  if (action == RUN) {
    m_main_state->getStateObject()->juryRunCommand(maneuver_entry);
    if(m_jury_state != COMLETE_SEND){
      m_jury_state = RUN_RECEIVED;
    }
  }
  else if (action == STOP) {
    m_main_state->getStateObject()->juryStopCommand();
    resetCarState();
    m_jury_state = STOP_RECEIVED;
  }
  else if (action == REQUEST_READY) {

    // After Stop command reset maneuver entry
    if((u_int32_t)maneuver_entry != m_system->getManeuver()->getCurrentManeuverId() && m_jury_state == STOP_RECEIVED) {
      resetCarState();
      m_jury_state = READY_RECEIVED;
    }

    m_system->getManeuver()->setManeuver(maneuver_entry);

    m_system->getPositionController()->go(StopReason::MANEUVER_COMPLETE);

    if(m_system->getLanetrackerJobManager()->isLTReady()) {
     m_system->callDriverState()(SendAction::READY,(int16_t)m_system->getManeuver()->getCurrentManeuverId());
    }
  }
}

bool MissionControl::getManeuver(int16_t& maneuver_entry)
{
  // not fully initalized
  if (m_jury_state == INIT)
  {
    maneuver_entry = -1;
    return false;
  }

  maneuver_entry = (int16_t)m_system->getManeuver()->getCurrentManeuver();
  return true;
}

void MissionControl::poseChanged(const PoseWithTime& new_pose)
{
  m_current_pose = new_pose;

  static u_int8_t frequency_counter = 0;

  // save new pose
  m_system->getMotionPlanning()->poseChanged(new_pose);
  m_system->getPositionController()->poseChanged(new_pose);

  // notify current state
  getStateObject()->poseChanged(new_pose);

  // pose is ca. 100 Hz, call timerUpdate with ca. 10 Hz
  if (frequency_counter % 10 == 0)
  {
    //frequency_counter = 0;

    //! Update the world by removing old Obstacles
    m_system->getWorld()->removeOldObstacles();

    // notify current state
    getStateObject()->timerUpdate();

    // notify other objects
    m_driving_strip_changer->timerCheck();

#ifdef KATANA_MC_SHOW_VEHICLE_POSE_INPUT
    std::cout <<"MissionControl - poseChanged: " <<new_pose.pose <<std::endl;
#endif
  }
  // call ca. 1 Hz
  if(frequency_counter % 100 == 0) {
    #ifdef MC_ENABLE_SVG_MAP
      if(m_system->mapOutdated() || getStateObject()->getState() == (u_int8_t) MainState::DRIVING) {
	// Write Map
	m_system->writeSVGMap();
      }
    #endif
    frequency_counter = 0;
  }
  frequency_counter++;
}

void MissionControl::newPatches(PatchVectorPtr patch_vector, PerceptionState perception_state)
{
  // register answer in lanetracker manager
  u_int32_t job_id;
  if (!m_system->getLanetrackerJobManager()->finishedJob(patch_vector, perception_state, job_id))   //< JobManager decides if canceled
    return;

  // notify state, if state returns true, add patches to world
  if (getStateObject()->lanetrackerCallback(job_id))
  {
    // add to world
    m_system->getWorld()->addNewPatches(patch_vector, perception_state);

    // notify state again, that patches have been added to world
    getStateObject()->newPatch(patch_vector->size(), perception_state, job_id);
  }

  #ifdef MC_ENABLE_SVG_MAP
  // outdate map
  if(getStateObject()->getState() == (u_int8_t) MainState::DRIVING) {
    m_system->outdateMap();
  }
  #endif
}

void MissionControl::newObstacles(sObstacle* obstacles, size_t number)
{
  //! Only store obstacles in State Driving
  if(getState() != MainState::DRIVING)
     return;

  const StateDriving::Ptr& main_state = static_pointer_cast<StateDriving>(getStateObject());

  for (size_t i = 0; i < number; i++)
  {
    getSystem()->getWorld()->addObstacle(std::make_shared<katana::Obstacle>(obstacles[i].toObstacle()));
  }

  // notify state
  getStateObject()->newObstacle();
}

void MissionControl::newTrafficSigns(sRoadSign* roadsigns, std::size_t number)
{
  for(size_t i = 0; i < number; i++) {
    // Skip not known traffic signs
    if(roadsigns[i].sign == TrafficSign::UNKNOWN) continue;

    RoadSign::Ptr sign = std::make_shared<katana::RoadSign>(roadsigns[i].toRoadSign());

    // Remember sign
    //m_system->getWorld()->saveRightofWay(sign->getSign());

    // Notify state
    getStateObject()->newTrafficSign(sign, roadsigns[i].signFound);
  }
}

void MissionControl::resetCarState()
{
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
  std::cout << "[MC] Resetting car state." << std::endl;
#endif

  // forget parking space type
  m_system->parkingSpaceType() = ParkingSpace::PARKING_SPACE_TYPE_COUNT;

  m_system->getWorld()->emptyPatches();
  m_system->getWorld()->emptyObstacles();

  // reset driving state
  m_main_state->changeState(MainState::WAITING);
  std::static_pointer_cast<StateDriving>(m_main_state->getStateContainer()[(u_int32_t)MainState::DRIVING])->resetStateMachine();

  // disable parking spot
  m_system->getWorld()->disableSearchForParkingSpot();

  // reset car lights
  m_system->getLightController()->resetLights();
  m_system->getLightController()->setLight(HEAD_LIGHT, true);

  #ifdef MC_ENABLE_SVG_MAP
  //Write map
  m_system->writeSVGMap();
  //Delete currently created map
  m_system->resetSVGMap();
  #endif

  // reset trajectory
  m_system->getPositionController()->clearTrajectory();

  // reset external triggers
  m_system->getManeuver()->resetExternalTrigger();
}

void MissionControl::debugTrigger(u_int8_t code)
{
  #ifdef KATANA_MC_GENERAL_DEBUG
    std::cout << "[MC] received external trigger code: " << (u_int32_t) code << std::endl;
  #endif

  if(code == 6) {
    #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout << "[MC] Chaning lane because of external trigger." << std::endl;;
    #endif
    m_driving_strip_changer->changeLane();
  } else if(code == 7) {
    #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout << "[MC] Drive right because of external trigger." << std::endl;;
    #endif
    m_system->getManeuver()->setExternalTrigger(Action::RIGHT);
  } else if(code == 8) {
    #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout << "[MC] Drive straight because of external trigger." << std::endl;;
    #endif
    m_system->getManeuver()->setExternalTrigger(Action::STRAIGHT);
  } else if(code == 9) {
    #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout << "[MC] Drive left because of external trigger." << std::endl;;
    #endif
    m_system->getManeuver()->setExternalTrigger(Action::LEFT);
  }
}

void MissionControl::cron(_clock_time current_streamTime)
{
  m_system->getWorld()->setStreamTime(current_streamTime);

  // Periodically (every 0,25 seconds) call jury running
  if(current_streamTime-m_last_stream_time > 250000) {
    // Check if we are in an unrecoverable error state
    if(m_main_state->getStateObject()->isError()) {
      m_system->callDriverState()(SendAction::ERROR,(int16_t)m_system->getManeuver()->getCurrentManeuverId());
    }
    // Only send running if request ready was answered before
    else if(m_jury_state == RUN_RECEIVED && m_system->getLanetrackerJobManager()->isLTReady()) {
      // Transmit current maneuver id Periodically
      m_system->callDriverState()(SendAction::RUNNING,(int16_t)m_system->getManeuver()->getCurrentManeuverId());
    }

    m_last_stream_time = current_streamTime;
  }
}



} //ns
