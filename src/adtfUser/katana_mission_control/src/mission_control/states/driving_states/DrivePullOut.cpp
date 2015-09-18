// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-10
 *
 */
//----------------------------------------------------------------------

#include "mission_control/states/driving_states/DrivePullOut.h"

#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
#include <iostream>
#endif

using namespace oadrive::core;

namespace katana
{

const Position2d DrivePullOut::STRAIGHT = Position2d(1.0, 0.0);

void DrivePullOut::onActivation(u_int32_t previous)
{
#ifdef KATANA_MC_STATE_DEBUG
  std::cout <<"MissionControl - DrivingState: PULLOUT" <<std::endl;
#endif

  // set pull out state
  m_state = PullOutState::WAITING_FOR_OBSTACLES;
  m_space = ParkingSpace::PARKING_SPACE_TYPE_COUNT;

  // stop vehicle
  m_system->getPositionController()->stop(StopReason::INITIALIZING);

  // take time
  m_waiting_start = std::chrono::steady_clock::now();

  // reset stuff
  m_best_matching_value = -std::numeric_limits<double>::infinity();

  // if parked in, get angle relative to street
  if (!m_system->getWorld()->getNextPatches().empty())
  {
    RoadBase::ConstPtr nearest_patch = m_system->getWorld()->getPatchNextToPosition(m_system->getPositionController()->getCurrentVehiclePose().translation());
    m_yaw_to_street = PoseTraits<Pose2d>::yaw(m_system->getPositionController()->getCurrentVehiclePose().inverse() * nearest_patch->getAnchorPose());

    m_system->getWorld()->emptyPatches();     //< clear world

    std::cout <<"yaw to street is: " <<m_yaw_to_street <<std::endl;
  }
  else
  {
    m_yaw_to_street = 0.0;
  }
}

bool DrivePullOut::onStateLeave(u_int32_t next_state)
{
  // stop blinking
  m_system->getLightController()->setLight(TURN_LEFT, false);
  m_system->getLightController()->setLight(TURN_RIGHT, false);

  // vision has found the street!
  // we are pulling out, increase maneuver
  // finished -> advance maneuver

  if (!m_system->getManeuver()->increaseManeuver())
  {
    m_change_state((u_int32_t)DrivingState::DRIVE_FINISHED);
    return false;
  }

  return true;
}


void DrivePullOut::sendSearchPatchToVision()
{
  assert(m_space != ParkingSpace::PARKING_SPACE_TYPE_COUNT);

  Pose2d start_pose;
  getEstimatedStartPose(start_pose, m_space, m_system->getManeuver()->getCurrentManeuver());

  // create first road patch (ID is one, because when parallel parking, pull out trajectory is 0!
  const RoadBase::Ptr initial_rb = m_system->getWorld()->instantiateRoadBase(PatchType::STRAIGHT, start_pose, 1);
  std::vector<katana::RoadBase::ConstPtr> vector(1, initial_rb);

  // create a second road patch to obtain longer trajectory to set reached pose
  Pose2d diff;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(diff, 0.5, 0.0, 0.0);
  const RoadBase::Ptr second = m_system->getWorld()->instantiateRoadBase(PatchType::STRAIGHT, start_pose * diff, 2);
  vector.push_back(second);

  // save target pose
  m_reached_pose = second->getDrivingStrip(DEFAULT_DRIVING_STRIP)->back().getPose();

  m_system->getLanetrackerJobManager()->submitJob(vector, INITIALIZE, 0, 0, -1.0);
}

void DrivePullOut::getEstimatedStartPose(Pose2d& p, ParkingSpace type, Action direction) const
{
  assert(type != ParkingSpace::PARKING_SPACE_TYPE_COUNT);

  if (type == ParkingSpace::CROSS)
  {
    const int8_t sign = direction == Action::PULL_OUT_LEFT ? 1 : -1;

    PoseTraits<Pose2d>::fromPositionAndOrientationRPY(p, 1.05, -0.15*sign, M_PI*0.5*sign);

    // world coordinates
    p = m_system->getPositionController()->getCurrentVehiclePose() * p;
  }
  else
  {
    // street pose already in world coordinates
    p = m_parallel_street_pose;
  }


}

bool DrivePullOut::lanetrackerCallback(u_int32_t job_id)
{
  if (m_state == PullOutState::WAITING_FOR_VISION_TO_CHECK)
  {

    u_int8_t i;
    for (i = 0; i < (u_int8_t)ParkingSpace::PARKING_SPACE_TYPE_COUNT; i++)
    {
      if (m_test_job_ids[i] == job_id)
        break;
    }
    if (i == (u_int8_t)ParkingSpace::PARKING_SPACE_TYPE_COUNT)  //< no job submitted by this state
    {
      return true;
    }

    if (parkingSpaceByVision(m_system->getLanetrackerJobManager()->getLastLTAnswer()->front().match_value, (ParkingSpace)i ) != ParkingSpace::PARKING_SPACE_TYPE_COUNT)
      beginSearchingForStreet();

    // do not add patch to world
    return false;
  }
  // vision used to search parallel street
  if (m_state == PullOutState::LOCALIZE_PARALLEL_STREET)
  {
    if (m_job_localize_patch == job_id)
    {

      // create parking patch
      m_parallel_street_pose = m_system->getLanetrackerJobManager()->getLastLTAnswer()->front().sp.toPose2dScaled();
      calculatePullOutPatchPose(m_pull_out_pose, m_parallel_street_pose);
      RoadBase::Ptr reverse_patch = m_system->getWorld()->instantiateRoadBase(PatchType::PARALLEL_PARKING_PULL_OUT_REVERSE, m_pull_out_pose, 0);

      // set state
      m_state = PullOutState::PULL_OUT_REVERSE;

      // set trajectory
      RoadPatch::TrajectoryPtr traj = reverse_patch->getDrivingStrip(DEFAULT_DRIVING_STRIP);
      m_system->getMotionPlanning()->prepareTrajectory(*traj);
      m_system->getPositionController()->setTrajectory(*reverse_patch->getDrivingStrip(DEFAULT_DRIVING_STRIP));

      // go
      m_system->getPositionController()->go(StopReason::INITIALIZING);

      // do not add street patch to world
      return false;
    }
  }
  return true;
}

void DrivePullOut::newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id)
{
  if (perception_state != PerceptionState::DETERMINING_PARKING_SPOT && m_state == WAITING_FOR_VISION_TO_CHECK)
  {
    // old answer from vision and we were sending something else...
    return;
  }

  Pose2d pull_out_reached_on_trajectory;

  switch(m_state)
  {
  case PullOutState::WAITING_FOR_OBSTACLES:
    // do nothing
    break;
  case PullOutState::WAITING_FOR_VISION_TO_CHECK:
  {
    // do nothing
    break;
  }
  case PullOutState::WAITING_FOR_LANETRACKER_TO_DRIVE:

    // SWITCH TO STANDARD DRIVING PROCEDURE, patches have already been added to world
    // set state
    m_state = PullOutState::DRIVING;

    // trigger lanetracker to track pull out patch (patches have already been added to world)
    m_system->getLanetrackerJobManager()->submitJob(m_system->getWorld()->getNextPatches(), NORMAL, 0, 0, -1.0);

    // generate trajectory
    m_system->updatePlannedMotion();

    // mark reached pose
    pull_out_reached_on_trajectory = m_system->getPositionController()->getNearestPoseOnCurrentTrajectory(m_reached_pose.translation());
    m_system->getPositionController()->setPositionToCheck(REACHED_POINT::RP_PULL_OUT, pull_out_reached_on_trajectory.translation(), 0.03);

    // go
    m_system->getPositionController()->go(StopReason::INITIALIZING);

    break;

  case PullOutState::DRIVING:

    // re-activate lanetracker
    const World::RoadPatchContainerConst next_patches = m_system->getWorld()->getNextPatches();
    m_system->getLanetrackerJobManager()->submitJob(next_patches, NORMAL, 0, 0, -1.0);

    break;
  }
}

void DrivePullOut::timerUpdate()
{
  switch(m_state)
  {
  case PullOutState::WAITING_FOR_OBSTACLES:
    // waited long enough?
    if (std::chrono::steady_clock::now() - WAITING_DURATION > m_waiting_start)
    {
      m_state = PullOutState::WAITING_FOR_LANETRACKER_TO_DRIVE;

      const Action command = m_system->getManeuver()->getCurrentManeuver();
      assert((command == Action::PULL_OUT_LEFT || command == Action::PULL_OUT_RIGHT) && "command == Action::PULL_OUT_LEFT || command == Action::PULL_OUT_RIGHT");

      if (command != Action::PULL_OUT_LEFT)
      {

        // if this flag is set, the vehicle has parked before and knows what kind of parking spot to expect
        if (m_system->parkingSpaceType() != ParkingSpace::PARKING_SPACE_TYPE_COUNT)
        {
          m_space = m_system->parkingSpaceType();
  #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
          std::cout <<"[PullOut] Remembered that ParkingSpace is type: " <<m_space <<std::endl;
  #endif
        }
        else
        {
          // check for parking space type
          m_space = getParkingSpaceTypeByObstacle();
          if (m_space == ParkingSpace::PARKING_SPACE_TYPE_COUNT)    //< no decision could be made, ask vision
          {
            m_state = PullOutState::WAITING_FOR_VISION_TO_CHECK;

            // submit test jobs to lanetracker
            tryParkingSpaces();

            // wait in new patches

            return;
          }

  #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
          std::cout <<"[PullOut] ParkingSpace by obstacles: " <<m_space <<std::endl;
  #endif
        }
      }
      else  //< pull_out_left -> this is easy
      {
        m_space = ParkingSpace::CROSS;

#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
        std::cout <<"[PullOut] ParkingSpace by maneuver: " <<m_space <<std::endl;
#endif
      }

      // parking space type determined
      // search for street
      beginSearchingForStreet();

    }
    break;
  case PullOutState::PULL_OUT_REVERSE:
  {
    if (m_system->getPositionController()->hasReached() ||
        m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_PULL_OUT_BEHIND, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
    {
      #ifdef KATANA_MC_PULL_OUT_DEBUG
      std::cout <<"[DrivePullOut] Now switching to forward trajectory" <<std::endl;
      #endif

      // Pose to drive out
      Pose2d pull_out_pose;
      PoseTraits<Pose2d>::fromPositionAndOrientationRPY(pull_out_pose, 0.0, 0.60, 0.0);

      // create forward patch and continue to drive as normal
      m_system->getWorld()->addRoadBase(PatchType::PARALLEL_PARKING_PULL_OUT_FORWARD,
                                        m_system->getPositionController()->getCurrentVehiclePose() * pull_out_pose, 0,
                                        m_system->getPositionController()->getOverallDrivenDistance());

      // set speed
      m_system->getMotionPlanning()->defaultSpeed() = 0.15;

      // let drive as normal
      beginSearchingForStreet();
    }

    break;
  }
  case PullOutState::WAITING_FOR_LANETRACKER_TO_DRIVE:
    // do nothing -> newPatches
    break;
  case PullOutState::WAITING_FOR_VISION_TO_CHECK:
    // do nothing -> newPatches
    break;
  case PullOutState::DRIVING:

    if (m_system->getPositionController()->hasReachedPosition(REACHED_POINT::RP_PULL_OUT))
    {
      // vehicle is on street, switch to normal driving
      m_change_state((u_int32_t)DrivingState::DRIVE_NORMAL);

      return;
    }

    // drive normal (same as normal driving)
    m_system->timerCheckIfBlocked();
    m_system->timerUpdateTrajectory();

    break;
  }
}

ParkingSpace DrivePullOut::getParkingSpaceTypeByObstacle() const
{

#ifdef KATANA_MC_PULL_OUT_DEBUG
  std::cout << "Obstacles in world: "  << m_system->getWorld()->getObstacleContainerPtr().size() << std::endl;

  if(m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_FRONT_OF_CAR, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
    std::cout << "isCrossParking Obstacle Front of Car" << std::endl;
  else
    std::cout << "isCrossParking NO Obstacle Front of Car" << std::endl;
  if(m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_BEHIND_CAR, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
    std::cout << "isCrossParking Obstacle Behind of Car" << std::endl;
  else
    std::cout << "isCrossParking NO Obstacle Behind of Car" << std::endl;
  if(m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_LEFT_SIDE_OF_CAR, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
    std::cout << "isCrossParking Obstacle Left Side of Car" << std::endl;
  else
    std::cout << "isCrossParking NO Obstacle Left Side of Car" << std::endl;
  if(m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_RIGHT_SIDE_OF_CAR, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
    std::cout << "isCrossParking Obstacle Right side of Car" << std::endl;
  else
    std::cout << "isCrossParking NO Obstacle Right side of Car" << std::endl;
#endif

  if(m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_FRONT_OF_CAR, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
  {
    return ParkingSpace::PARALLEL;
  }
  else if(m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_LEFT_SIDE_OF_CAR, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
  {
    return ParkingSpace::CROSS;
  }

  // this is difficult
  return ParkingSpace::PARKING_SPACE_TYPE_COUNT;
}

void DrivePullOut::tryParkingSpaces()
{
  m_answers_from_lanetracker = 0;


  // let vision decide, first cross parking
  for (u_int8_t i = 0; i < (u_int8_t)ParkingSpace::PARKING_SPACE_TYPE_COUNT; i++)
  {
    // trigger vision
    m_test_job_ids[i] = localizeStreet((ParkingSpace)i);
  }
}

u_int32_t DrivePullOut::localizeStreet(ParkingSpace type)
{
  Pose2d start_pose;

  if (type == ParkingSpace::CROSS)
  {
    // search cross patch to the left of car to prevent from finding parallel parking lines
    PoseTraits<Pose2d>::fromPositionAndOrientationRPY(start_pose, 1.05, 1.1, -M_PI*0.5);
  }
  else
  {
    Pose2d rotate;
    // *0.5 -> hack because position is imprecise
    PoseTraits<Pose2d>::fromPositionAndOrientationRPY(rotate, 0.0, 0.0, m_yaw_to_street*0.5);

    PoseTraits<Pose2d>::fromPositionAndOrientationRPY(start_pose, 1.4, 0.7, 0.0);

    start_pose = rotate * start_pose;
  }

  // world coordinates
  start_pose = m_system->getPositionController()->getCurrentVehiclePose() * start_pose;

  // create first road patch
  const RoadBase::Ptr initial_rb = m_system->getWorld()->instantiateRoadBase(PatchType::STRAIGHT, start_pose, 0);
  std::vector<katana::RoadBase::ConstPtr> vector(1, initial_rb);

  return m_system->getLanetrackerJobManager()->submitJob(vector, DETERMINING_PARKING_SPOT, 1, oadrive::vision::PatchesToLookFor::STRAIGHTS, 0.0);
}

ParkingSpace DrivePullOut::parkingSpaceByVision(double match_value, ParkingSpace type)
{

#ifdef KATANA_MC_PULL_OUT_DEBUG
  std::cout <<"[PullOut] Match value for option " <<(u_int32_t)type <<": " <<match_value <<std::endl;
#endif

  if (match_value > m_best_matching_value)
  {
    m_best_matching_value = match_value;
    m_best_option = type;
  }

  ++m_answers_from_lanetracker;

  if (m_answers_from_lanetracker == (u_int8_t)ParkingSpace::PARKING_SPACE_TYPE_COUNT)    //< tried everything
  {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
    std::cout <<"[PullOut] ParkingSpace by vision: " <<m_best_option <<", match value: " <<m_best_matching_value <<std::endl;
#endif

    m_space = m_best_option;

    return m_best_option;
  }

  return ParkingSpace::PARKING_SPACE_TYPE_COUNT;                  //< need continue to search
}

void DrivePullOut::beginSearchingForStreet()
{
  // blinking
  turnOnTurnSignals();

  if (m_space != ParkingSpace::CROSS)    //< much harder to pull out
  {
    if (m_state != PULL_OUT_REVERSE)    //< if not already in this state
    {
      // slow down
      m_system->getMotionPlanning()->defaultSpeed() = 0.10;

      // first: localize street to place trajectories
      m_state = PullOutState::LOCALIZE_PARALLEL_STREET;
      m_job_localize_patch = localizeStreet(ParkingSpace::PARALLEL);
      return;
    }
  }

  // now lanetracker searching for street, normal driving
  m_state = PullOutState::WAITING_FOR_LANETRACKER_TO_DRIVE;

  // trigger vision, set reached pose where to change to normal
  sendSearchPatchToVision();
}

void DrivePullOut::calculatePullOutPatchPose(Pose2d &patch_pose, const Pose2d street_patch_localization) const
{
  // Project the current vehicle pose on extension of street localization
  const Position2d street = street_patch_localization.rotation() * STRAIGHT;
  const Position2d to_vehicle = m_system->getPositionController()->getCurrentVehiclePose().translation() - street_patch_localization.translation();

  // Projection
  const double t = to_vehicle.dot(street)/street.squaredNorm();

  // set patch pose
  patch_pose = street_patch_localization;
  patch_pose.translation() += t * street;

#ifdef KATANA_MC_PULL_OUT_DEBUG
  std::cout <<"[PullOut] Street patch is at: " <<street_patch_localization <<std::endl;
  std::cout <<"[PullOut] Setting pull out patch to pose: " <<patch_pose <<std::endl;
#endif
}

void DrivePullOut::turnOnTurnSignals()
{
  // start blinking
  const bool left_blink = (m_space == ParkingSpace::PARALLEL) || (m_system->getManeuver()->getCurrentManeuver() == Action::PULL_OUT_LEFT);

  if(left_blink)
    m_system->getLightController()->setLight(LightName::TURN_LEFT, true);
  else
    m_system->getLightController()->setLight(LightName::TURN_RIGHT, true);
}

} // ns
