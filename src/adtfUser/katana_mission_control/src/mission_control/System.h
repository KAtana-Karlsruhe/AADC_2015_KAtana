// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-17
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_SYSTEM_H
#define _MISSION_CONTROL_SYSTEM_H

#include "katanaCommon/katanaCommon.h"
#include "Pose.h"

#include "mission_control/world.h"
#include "mission_control/MotionPlanning.h"
#include "mission_control/maneuver.h"
#include "mission_control/PositionController.h"
#include "mission_control/LightController.h"
#include "mission_control/CollisionHandler.h"
#include "mission_control/LanetrackerJobManager.h"
#include "helperClasses/Config.h"

#include <oadrive_vision/Matcher.h>

namespace katana
{

typedef int16_t Maneuver_Entry;
//! DriverState transmit function
typedef std::function<void(SendAction action, Maneuver_Entry maneuver)> TransmitDriverStateFunc;

class System
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<System> Ptr;

  //! Constructor
  System();

  //! Destructor
  virtual ~System()     {}

  /** ******************* INIT / GETTER / ACCESS **********************/
  //! Init
  bool Initialize(const std::string& maneuver_config,
                  const std::string& patch_config,
                  const std::string& mc_config,
                  FuncSteering call_output_steering,
                  FuncTorque call_output_speed,
                  TransmitPatchesFunc call_output_patches,
                  TransmitStatusFunc call_output_status,
                  TransmitDriverStateFunc call_output_driver_state,
                  LightStateFunc call_output_lights
                  );

  //! Access
  const World::Ptr& getWorld() const                              { return m_world; }
  const Maneuver::Ptr& getManeuver() const                        { return m_maneuver; }
  const MotionPlanning::Ptr& getMotionPlanning() const            { return m_motionplanning; }
  const PositionController::Ptr& getPositionController() const    { return m_position_controller; }
  const CollisionHandler::Ptr& getCollisionHandler() const        { return m_collision_handler; }
  const LightController::Ptr& getLightController() const          { return m_light_controller; }
  const LanetrackerJobManager::Ptr& getLanetrackerJobManager() const    { return m_lt_manager; }
  const Config::Ptr& getConfig() const                            { return m_config; }

  //! Remember parking space
  const ParkingSpace& parkingSpaceType() const                    { return m_parking_space_type; }
  ParkingSpace& parkingSpaceType()                                { return m_parking_space_type; }

  //! Time since blocked by obstacle
  u_int32_t getBlockedByObstacleCounter() const                   { return m_blocked_by_obstacle_counter; }
  void resetBlockedByObstacleCounter()                            { m_blocked_by_obstacle_counter = 0; }

  //! Transmit
  const TransmitStatusFunc& callOutputStatus() const { return m_output_status; }
//  const LightStateFunc& callOutputLight() const                                 { return m_output_light; }

  const TransmitDriverStateFunc& callDriverState() const			{ return m_output_driver_state; }
  TransmitDriverStateFunc m_output_driver_state;

  /** ********************* CONTROL ***********************/

  //! Trigger Lanetracker to search with "initial" method
  void callLanetrackerInitial(u_int8_t number_of_stitches, oadrive::vision::PatchesToLookFor patches_to_look_for, const double& matching_threshold) const;

  //! Get a straight patch unterneatch the car
  RoadBase::Ptr getPatchUnderCar() const;

  /**
  * @brief updatePlannedMotion
  * 1. Get next patches from world according to current position
  * 2. Let motion planning create the extended output points from patches
  * 3. Transmit new points to lateral controller
  * 4. Returns false if trajectory has less then 3 points
  */
  bool updatePlannedMotion() const;
  bool updatePlannedMotion(World::RoadPatchContainerConst patches) const;

  //! Check if a traffic sign is a junction sign
  bool isJunctionSign(RoadSign::Ptr sign) const;

  //! Reset counter which checks if car is blocked
  void resetBlockedArduinoCounter()        { m_blocked_retry_counter = BLOCKED_RETRY_TIMESPAN+1; }

  /** ************** TIMER needed for DRIVING ******************/
  //! Call in timer update to checked for blocked car
  void timerCheckIfBlocked();
  //! Call in timer update to check for obstacles in front of car
  bool timerCheckIfObstacleInFront();
  bool timerCheckIfObstacleOnTrajectory();
  bool timerCheckIfObstacleInFront(Obstacle::Ptr obs);
  Obstacle::Ptr checkObstacleTrajectory();
  static oadrive::core::Pose2d VEHICLE_FRONT_POSE;
  //! Check if driving 25 cm, update trajectory
  void timerUpdateTrajectory();

  /** ************** STATE change **************************/
  DrivingState getNextState(RoadSign::Ptr sign);

  /** ************** SVG map *******************************/
  #ifdef MC_ENABLE_SVG_MAP
  void writeSVGMap();
  void resetSVGMap();
  bool mapOutdated()	{ return m_map_outdated; }
  void outdateMap()	{ m_map_outdated = true; }
  #endif

private:
  //! Modules
  Config::Ptr m_config;
  World::Ptr m_world;
  Maneuver::Ptr m_maneuver;
  MotionPlanning::Ptr m_motionplanning;
  PositionController::Ptr m_position_controller;
  LightController::Ptr m_light_controller;
  CollisionHandler::Ptr m_collision_handler;
  LanetrackerJobManager::Ptr m_lt_manager;

  //! Output status
  TransmitStatusFunc m_output_status;

  /** ****************** CONTROL ******************/

  //! Counter to retry driving when vehicle is blocked, e.g. arduino restart
  u_int32_t m_blocked_retry_counter;

  //! Warning that sign doesn't mathc maneuver entry
  bool m_sign_warning_send;

  //! Wait when blocked
  static constexpr u_int32_t BLOCKED_RETRY_TIMESPAN = 40;     //< wait ca. 4 secs

  //! Minimum area that is required for a most relevant sign to trigger the junction state
  float m_min_roadsign_junction_change;

  float m_min_parking_sign_change;

  //! Remember parking position when parked in by vehicle
  ParkingSpace m_parking_space_type;

  //! Timer for blocked by obstacle status
  u_int32_t m_blocked_by_obstacle_counter;

  #ifdef MC_ENABLE_SVG_MAP
  //! All current patches added to map?
  bool m_map_outdated;
  #endif
  #ifdef MC_ENABLE_SVG_MAP_COUNTER
  int m_map_counter;
  #endif
};


} // ns

#endif //_MISSION_CONTROL_SYSTEM_H
