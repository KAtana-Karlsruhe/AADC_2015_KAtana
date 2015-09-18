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

#include "System.h"

using namespace oadrive::core;

namespace katana
{

Pose2d System::VEHICLE_FRONT_POSE;

System::System()
  : m_blocked_retry_counter(BLOCKED_RETRY_TIMESPAN+1)
  , m_sign_warning_send(false)
  , m_blocked_by_obstacle_counter(0)
  #ifdef MC_ENABLE_SVG_MAP
  , m_map_outdated(false)
  #endif
  #ifdef MC_ENABLE_SVG_MAP_COUNTER
  , m_map_counter(0)
  #endif
{
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(VEHICLE_FRONT_POSE, 0.5, 0.0, 0.0);
}

bool System::Initialize(const std::string& maneuver_config,
                        const string& patch_config,
                        const std::string& mc_config,
                        FuncSteering call_output_steering,
                        FuncTorque call_output_speed,
                        TransmitPatchesFunc call_output_patches,
                        TransmitStatusFunc call_output_status,
                        TransmitDriverStateFunc call_output_driver_state,
                        LightStateFunc call_output_lights
                        )
{
  assert(call_output_patches);
  assert(call_output_status);
  assert(call_output_steering);
  assert(call_output_speed);
  assert(call_output_driver_state);
  assert(call_output_lights);

  // config
  m_config.reset(new Config);
  if (!m_config->readFromFile(mc_config))
    return false;

  m_min_roadsign_junction_change = m_config->getDouble(CONF_MIN_ROADSIGN_JUNCTION_CHANGE);
  m_min_parking_sign_change = m_config->getDouble(CONF_MIN_PARKING_SIGN_CHANGE);

  // Parking space type when vehicle is parked
  m_parking_space_type = ParkingSpace::PARKING_SPACE_TYPE_COUNT;    //< unknown at beginning

  // Function pointers
  m_output_status = call_output_status;
  m_output_driver_state = call_output_driver_state;

  // Load actions/maneuver/sectors
  m_maneuver.reset();
  m_maneuver = std::make_shared<Maneuver>();
  m_maneuver->readManeuver(maneuver_config);

  // lt job manager
  m_lt_manager.reset(new LanetrackerJobManager(call_output_patches));

  // light controller
  m_light_controller.reset(new LightController(call_output_lights));

  // Create motion planner
  m_motionplanning.reset(new MotionPlanning);
  m_motionplanning->defaultSpeed() = m_config->getDouble(DoubleParameter::CONF_STANDARD_VELOCITY);

  // Create position controller
  m_position_controller.reset(new PositionController(call_output_steering, call_output_speed, call_output_status));
  m_position_controller->initialize(m_light_controller, m_config);    //< brake light

  // Create world instance
  m_world.reset(new World());
  m_world->initialize(m_position_controller, m_maneuver);
  m_world->loadPatches(patch_config);

  // Collision handler
  m_collision_handler.reset( new CollisionHandler(m_world));

  return true;
}

void System::callLanetrackerInitial(u_int8_t number_of_stitches, oadrive::vision::PatchesToLookFor patches_to_look_for, const double& matching_threshold) const
{
  const katana::World::RoadPatchContainerConst& patches = getWorld()->getNextPatches();

  RoadBase::ConstPtr guess_patch;

  // we are starting...
  if (patches.empty())
  {
    guess_patch = getPatchUnderCar();
  }
  else  // we've been driven already, repeat last patch
  {
    guess_patch = patches.back();
  }

  std::vector<katana::RoadBase::ConstPtr> vector(1, guess_patch);
  getLanetrackerJobManager()->submitJob(vector, INITIALIZE, 0, 0, -1.0);
}

RoadBase::Ptr System::getPatchUnderCar() const
{
  // create a straight patch underneath car
  Pose2d relative_to_car;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(relative_to_car, 0, 0.23, 0);
  Pose2d patch_anchor = getPositionController()->getCurrentVehiclePose() * relative_to_car;

  return std::make_shared<RoadBase>((getWorld()->getPatchTemplates())[(u_int32_t)PatchType::STRAIGHT], patch_anchor, 0);
}


bool System::updatePlannedMotion() const
{
  return updatePlannedMotion(getWorld()->getNextPatches());
}


bool System::updatePlannedMotion(World::RoadPatchContainerConst patches) const
{
  // let world remove old patches
  getWorld()->removeOldPatches(getPositionController()->getOverallDrivenDistance());

  // Connect patches and create new / overwrite extended points
  // Try to generate a new driving strip
  getMotionPlanning()->generateRoadDrivingTrajectory(patches);

  // set flag that current patches have been used to create trajectory
  getWorld()->trajectoryGeneratedFromPatches();

  if (getMotionPlanning()->getTrajectory().size() < MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER)
  {
    return false;
  }

  // Overwrite current driving trajectory
  getPositionController()->setTrajectory(getMotionPlanning()->getTrajectory());

#ifdef MC_ENABLE_SVG_MAP
  getWorld()->getMapWriter().addTrajectory(getMotionPlanning()->getTrajectory());
#endif

  return true;
}

bool System::isJunctionSign(RoadSign::Ptr sign) const
{
  if(
    sign->getSign() == TrafficSign::JUNCTION_PRIORITY_FROM_RIGHT ||
    sign->getSign() == TrafficSign::JUNCTION_GIVE_WAY ||
    sign->getSign() == TrafficSign::JUNCTION_STOP_GIVE_WAY ||
    sign->getSign() == TrafficSign::JUNCTION_PRIORITY
    ) {
    return true;
  }

  return false;
}

void System::timerCheckIfBlocked()
{
  /**********************************************************************************************/
  /************** THIS FOR CHECKING IF THE CAR IS BLOCKED OR IN BLOCKED WAITING STATE ***********/
  /**********************************************************************************************/
  if (m_blocked_retry_counter <= BLOCKED_RETRY_TIMESPAN)  //< state is in waiting to retry state (counting down...)
  {
    --m_blocked_retry_counter;

    // have waited for a long enough time? try to drive again
    if (m_blocked_retry_counter == 0)
    {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout <<"[MissionControl] TRYING TO DRIVE AGAIN" <<std::endl;
#endif
      getPositionController()->setNonBlocked();
      m_blocked_retry_counter = BLOCKED_RETRY_TIMESPAN+1;   //< set to non-blocked counter value
    }
  }
  else if (getPositionController()->isBlocked())    //< vehicle is blocked
  {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
    std::cout <<"[MissionControl] VEHICLE_BLOCKED - waiting..." <<std::endl;
#endif
    m_blocked_retry_counter = BLOCKED_RETRY_TIMESPAN;   //< start counting down
  }
  /*********************************************************************************************/
}

bool System::timerCheckIfObstacleInFront() {
  Obstacle::Ptr obs = getCollisionHandler()->collisionDriving(getPositionController()->getCurrentVehiclePose(), getPositionController()->getCurrentSteering());
  return timerCheckIfObstacleInFront(obs);
}


bool System::timerCheckIfObstacleInFront(Obstacle::Ptr obs)
{
  //! Stopping the car if there is an obstacle in front
//  Obstacle::Ptr obs = getCollisionHandler()->collision(katana::CollisionHandler::BOX_IN_FRONT_OF_CAR, getPositionController()->getCurrentVehiclePose());
  if(obs != nullptr)
  {
    if (!getPositionController()->mustNotMove(katana::StopReason::OBSTACLE)) //< not already blocked by obstacle
    {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      if(obs->getSource() == ObstacleSource::IR_FRONT_CENTER_SHORT)
      {
        std::cout << "[Obstacles] Car Blocking IR FRONT SHORT" << std::endl;
      }else if(obs->getSource() == ObstacleSource::IR_FRONT_CENTER_LONG)
      {
          std::cout << "[Obstacles] Car Blocking IR FRONT LONG" << std::endl;
      }else if(obs->getSource() == ObstacleSource::US_FRONT_LEFT)
      {
          std::cout << "[Obstacles] Car Blocking US FRONT LEFT" << std::endl;
      }else if(obs->getSource() == ObstacleSource::US_FRONT_RIGHT)
      {
          std::cout << "[Obstacles] Car Blocking US FRONT RIGHT" << std::endl;
      }
#endif

      getPositionController()->stop(katana::StopReason::OBSTACLE);
    }
    ++m_blocked_by_obstacle_counter;
    return true;
  }
  else if (getPositionController()->mustNotMove(katana::StopReason::OBSTACLE))
  {
    getPositionController()->go(katana::StopReason::OBSTACLE);
    m_blocked_by_obstacle_counter = 0;
  }
  return false;
}

bool System::timerCheckIfObstacleOnTrajectory()
{
  Obstacle::Ptr obs = checkObstacleTrajectory();
  if(obs != nullptr)
  {
    if (!getPositionController()->mustNotMove(katana::StopReason::OBSTACLE)) //< not already blocked by obstacle
    {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout <<"[System] Block because obstacle on trajectory" <<std::endl;
#endif

      getPositionController()->stop(katana::StopReason::OBSTACLE);
    }
    ++m_blocked_by_obstacle_counter;
    return true;
  }
  else if (getPositionController()->mustNotMove(katana::StopReason::OBSTACLE))
  {
    getPositionController()->go(katana::StopReason::OBSTACLE);
    m_blocked_by_obstacle_counter = 0;
  }
  return false;
}

Obstacle::Ptr System::checkObstacleTrajectory()
{
  // no trajectory... no change for collision
  if (getPositionController()->getTrajectory().empty())
    return nullptr;

  const Pose2d front = getPositionController()->getCurrentVehiclePose() * VEHICLE_FRONT_POSE;
  const std::size_t start_index = getPositionController()->getNearestIndexOnTrajectory(getPositionController()->getTrajectory(), front.translation());

  return getCollisionHandler()->collisionTrajectory(getPositionController()->getTrajectory(),
                                                    start_index,
                                                    0.4,
                                                    2,
                                                    CollisionHandler::BOX_CAR_ON_TRAJECTORY);
}

DrivingState System::getNextState(RoadSign::Ptr sign)
{
 const Action currentManeuver =getManeuver()->getCurrentManeuver();

 // Change to parking state if sign reaches certain area
  if(sign->getSign() == TrafficSign::PARKING_AHEAD && sign->getArea() > m_min_parking_sign_change) {
    if(currentManeuver == Action::CROSS_PARKING || currentManeuver == Action::PARALLEL_PARKING)
    {
      #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
	  std::cout <<"[DriveNormal] Recognized traffic sign, changing to PARKING" <<std::endl;
      #endif

      m_sign_warning_send = false;

      // Parking sign seen -> change in parking state
      return DrivingState::DRIVE_PARKING;
    }
    #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
    else if(!m_sign_warning_send) {
      std::cout << "MC WARNING: Found parking sign, but current maneuver is not parking." << std::endl;
      m_sign_warning_send = true;
    }
    #endif
  }
  // Change to junction state if sign reaches a certain area
  else if(isJunctionSign(sign) && sign->getArea() > m_min_roadsign_junction_change) {
    if(currentManeuver == Action::LEFT || currentManeuver == Action::RIGHT || currentManeuver == Action::STRAIGHT)
    {
      #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
	  std::cout <<"[DriveNormal] Recognized traffic sign, chaning to JUNCTION" <<std::endl;
      #endif

      m_sign_warning_send = false;

      return DrivingState::DRIVE_JUNCTION;
    }
    #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
    else if (!m_sign_warning_send) {
      std::cout << "MC WARNING: Found junction sign, but current maneuver is not a junction maneuver." << std::endl;
      m_sign_warning_send = true;
    }
    #endif
  }
  return DrivingState::DRIVE_STATE_COUNT;
}

void System::timerUpdateTrajectory()
{
  // Check for driven distance and update planning
  // if hasReached() is true, also try to generate a new driving strip
  if (
      (getMotionPlanning()->getDistanceSinceLastUpdate() > DRIVING_DISTANCE_TO_DRIVING_STRIP_UPDATE  //< we have driven a sufficiant distance, so we need an update,
       && getWorld()->havePatchesBeenUpdated())        //< but only if new patches are available; otherwise, delay update until new patches are available

      || getPositionController()->hasReached()  )      //< we are at end of driving strip, always try to generate new
  {
    updatePlannedMotion();
  }
}

#ifdef MC_ENABLE_SVG_MAP
void System::writeSVGMap()
{
  if(!m_map_outdated) return;

  // write left over patches
  const katana::World::RoadPatchContainerConst& patches = getWorld()->getNextPatches();
  for (World::RoadPatchContainerConst::const_iterator it = patches.begin(); it != patches.end(); it++)
  {
    getWorld()->addPatchToMap(**it);
  }

  // write image -> todo: change filename (increasing number?)
  getWorld()->getMapWriter().write("/tmp/localMap/local_map.svg");

  #ifdef MC_ENABLE_SVG_MAP_COUNTER
    getWorld()->getMapWriter().write("/tmp/localMap/local_map" + std::to_string(m_map_counter) + ".svg");
    ++m_map_counter;
  #endif

  m_map_outdated = false;
}

void System::resetSVGMap()
{
  // clear everything
  getWorld()->getMapWriter().clear();
}
#endif


} // ns
