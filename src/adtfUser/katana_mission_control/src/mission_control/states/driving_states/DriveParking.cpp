#include "DriveParking.h"

using namespace oadrive::core;

namespace katana {

void DriveParking::onActivation(u_int32_t previous)
{
  #ifdef KATANA_MC_STATE_DEBUG
    std::cout <<"MissionControl - DrivingState: PARKING" <<std::endl;
  #endif

  // set state
  m_state = ParkingState::SEARCHING;

  // speed
  m_system->getMotionPlanning()->defaultSpeed() = 0.15;

  const Action command = m_system->getManeuver()->getCurrentManeuver();

  assert((command == Action::CROSS_PARKING || command == Action::PARALLEL_PARKING) && "command == Action::CROSS_PARKING || command == Action::PARALLEL_PARKING");

  // type of parking space to look for
  ParkingSpace ps = command == Action::CROSS_PARKING ? ParkingSpace::CROSS : ParkingSpace::PARALLEL;

  // world automaticly saves all obstalces when in state parking or pull-out
  // let parking assistance serach for parking space
  m_parking_assistant = m_system->getWorld()->searchForParkingSpot(ps);

  m_distance_checked = m_system->getPositionController()->getOverallDrivenDistance();
}

bool DriveParking::onStateLeave(u_int32_t next_state)
{
  // finished -> advance maneuver
  if (!m_system->getManeuver()->increaseManeuver())
  {
    m_change_state((u_int32_t)DrivingState::DRIVE_FINISHED);
    return false;
  }
  return true;
}


void DriveParking::timerUpdate()
{
  switch(m_state)
  {
  case ParkingState::SEARCHING:

    // check if vehicle is physically blocked, e.g. after arduino crash
    m_system->timerCheckIfBlocked();

    // Check for driven distance and update planning
    // if hasReached() is true, also try to generate a new driving strip
    m_system->timerUpdateTrajectory();

    //
    if (m_system->getPositionController()->getOverallDrivenDistance() - CHECK_FOR_SPOT_DISTANCE > m_distance_checked)
    {
      // remember this position
      m_distance_checked = m_system->getPositionController()->getOverallDrivenDistance();
      // let park assistant check if an empty parking spot is found
      m_parking_assistant->evaluateMeasurments(m_system->getPositionController()->getOverallDrivenDistance());
    }

    // check if found parking spot
    double beginning_empty;
    if (m_parking_assistant->foundAppropriateSpot(beginning_empty))
    {
      // stop searching
      m_system->getWorld()->disableSearchForParkingSpot();

      // start blinking
      m_system->getLightController()->setLight(LightName::TURN_RIGHT, true);

      // start parking maneuver
      const double diff = beginning_empty - m_system->getPositionController()->getOverallDrivenDistance();

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"Starting to reverse, beginning of parking spot is at " <<diff <<" m." <<std::endl;
#endif
      // calculate pose of parking patch
      m_patch_pose = getParkingPatchPose(diff);
#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"[DriveParking] Estimated parking patch pose is: " <<m_patch_pose <<std::endl;
#endif


      m_state = ParkingState::FORWARD;

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"Now in state Forward1" <<std::endl;
#endif

      forwardMove(m_patch_pose, m_parking_assistant->getParkingType());

    }

    break;
  case ParkingState::FORWARD:
    // check if vehicle is physically blocked, e.g. after arduino crash
    m_system->timerCheckIfBlocked();

    if (m_system->getPositionController()->hasReached())
    {
      m_state = ParkingState::REVERSING;

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"Now in state Reverse" <<std::endl;
#endif

      reversePark(m_patch_pose, m_parking_assistant->getParkingType());

    }

    break;
  case ParkingState::REVERSING:

    // check if vehicle is physically blocked, e.g. after arduino crash
    m_system->timerCheckIfBlocked();

    if (m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_SMALL_BEHIND_OF_CAR, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
    {
      m_system->getPositionController()->stop(StopReason::OBSTACLE);

      // stop blinking
      m_system->getLightController()->setLight(LightName::TURN_RIGHT, false);

      // switch state
      m_state = ParkingState::FORWARD2;

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"Now in state Forward2 due to obstacle behind" <<std::endl;
#endif

      forwardMove(m_patch_pose, m_parking_assistant->getParkingType());

      m_system->getPositionController()->go(StopReason::OBSTACLE);

    }
    else if (m_system->getPositionController()->hasReached())
    {
      // stop blinking
      m_system->getLightController()->setLight(LightName::TURN_RIGHT, false);


      // switch state
      m_state = ParkingState::FORWARD2;

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"Now in state Forward2 due to reached trajectory" <<std::endl;
#endif

      forwardMove(m_patch_pose, m_parking_assistant->getParkingType());

    }

    break;
  case ParkingState::FORWARD2:
    // check if vehicle is physically blocked, e.g. after arduino crash
    m_system->timerCheckIfBlocked();

    if (m_system->getPositionController()->hasReached() ||
        m_system->getCollisionHandler()->collision(CollisionHandler::BOX_PARK_SMALL_FRONT_OF_CAR, m_system->getPositionController()->getCurrentVehiclePose()) != nullptr)
    {
      m_system->getPositionController()->stop(StopReason::OBSTACLE);

      // start warning lights
      m_system->getLightController()->setLight(LightName::WARNING_LIGHTS, true);

      m_state = ParkingState::WAITING;

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"Now in state Waiting" <<std::endl;
#endif

      // take time
      m_waiting_start = std::chrono::steady_clock::now();

      m_system->getPositionController()->go(StopReason::OBSTACLE);
    }

    break;
  case ParkingState::WAITING:

    if (std::chrono::steady_clock::now() - m_waiting_start > WAITING_DURATION)
    {
#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"Vehicle position: " <<m_patch_pose.inverse()*m_system->getPositionController()->getCurrentVehiclePose() <<std::endl;
#endif

      // stop warning lights
      m_system->getLightController()->setLight(LightName::WARNING_LIGHTS, false);

      // finished parking, remember parking space type
      m_system->parkingSpaceType() = m_parking_assistant->getParkingType();

      m_change_state((u_int32_t)DrivingState::DRIVE_PULLOUT);
    }

    break;
  }

  // Failsafe???

  // blinking

  // mark maneuver as done

  // change to initialize
}

void DriveParking::reversePark(const Pose2d& patch_pose, ParkingSpace type)
{
  // set to slower speed
  m_system->getMotionPlanning()->defaultSpeed() = 0.15;

  RoadBase::ConstPtr parking_patch;

  if (type == ParkingSpace::CROSS)
  {
    parking_patch = m_system->getWorld()->instantiateRoadBase(PatchType::CROSS_PARKING_REVERSE,
                                                                  patch_pose,
                                                                  0);
  }
  else
  {
    parking_patch = m_system->getWorld()->instantiateRoadBase(PatchType::PARALLEL_PARKING_REVERSE,
                                                                  patch_pose,
                                                                  0);
  }

  World::RoadPatchContainerConst tmp;
  tmp.push_back(parking_patch);

  m_system->getMotionPlanning()->generateRoadDrivingTrajectory(tmp);

  m_system->getPositionController()->setTrajectory(m_system->getMotionPlanning()->getTrajectory());
}

void DriveParking::forwardMove(const Pose2d &patch_pose, ParkingSpace type)
{
  RoadBase::ConstPtr parking_patch;

  //No need to get forward in case of cross parking
  if(m_state == ParkingState::FORWARD2 && type == ParkingSpace::CROSS)
    return;

  if (type == ParkingSpace::CROSS)
  {
    parking_patch = m_system->getWorld()->instantiateRoadBase(PatchType::CROSS_PARKING_FORWARD,
                                                                  patch_pose,
                                                                  0);
  }
  else
  {
    if(m_state == ParkingState::FORWARD)
    {
      parking_patch = m_system->getWorld()->instantiateRoadBase(PatchType::PARALLEL_PARKING_FORWARD,
                                                                  patch_pose,
                                                                  0);
    } else if(m_state == ParkingState::FORWARD2)
    {
#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
      std::cout <<"Now in state Forward2 and should normally go forward" <<std::endl;
#endif

        m_system->getMotionPlanning()->defaultSpeed() = 0.10;

        parking_patch = m_system->getWorld()->instantiateRoadBase(PatchType::PARALLEL_PARKING_FORWARD_2,
                                                                    patch_pose,
                                                                    0);
    }
  }

  World::RoadPatchContainerConst tmp;
  tmp.push_back(parking_patch);

  m_system->getMotionPlanning()->generateRoadDrivingTrajectory(tmp);

  m_system->getPositionController()->setTrajectory(m_system->getMotionPlanning()->getTrajectory());
}

void DriveParking::newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id)
{
  switch(m_state)
  {
  case ParkingState::SEARCHING:
    // lanetracker didn't see anything, we are at end of trajectory -> change to initialise
    if (num == 0 && m_system->getPositionController()->hasReached())
    {
      // trigger as usual
      m_system->getLanetrackerJobManager()->submitJob(m_system->getWorld()->getNextPatches(), NORMAL, 0, 0, -1.0);

      // add one straight patch
      // id:
      u_int32_t next_id;
      const World::RoadPatchContainerConst patches = m_system->getWorld()->getNextPatches();
      if (patches.empty())
      {
        next_id = 0;
      }
      else
      {
        next_id = patches.back()->getId() + 1;
      }

      Pose2d before_car;
      PoseTraits<Pose2d>::fromPositionAndOrientationRPY(before_car, 0.8, 0.0, 0.0);
      m_system->getWorld()->addRoadBase(PatchType::STRAIGHT, m_system->getPositionController()->getCurrentVehiclePose() * before_car, next_id,
                                        m_system->getPositionController()->getOverallDrivenDistance());

      m_system->updatePlannedMotion();
    }
    else
    {
      m_system->getLanetrackerJobManager()->submitJob(m_system->getWorld()->getNextPatches(), NORMAL, 0, 0, -1.0);
    }
    break;
  default:
    // do not re-activate the lanetracker
    break;
  }

}

Pose2d DriveParking::getParkingPatchPose(const double &diff_distance) const
{
  Pose2d diff;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(diff, diff_distance, 0.23, 0.0);

  return m_system->getPositionController()->getCurrentVehiclePose() * diff;

  /*const Pose2d behind = m_system->getPositionController()->getCurrentVehiclePose() * diff;

  Pose2d nearest_current = m_system->getPositionController()->getProjectedPose().getPose();
  Pose2d nearest_behind = m_system->getPositionController()->getNearestPoseOnCurrentTrajectory(behind.translation());

  ExtendedPose2d ret(nearest_behind);
  ret.directTo(nearest_current.translation());

  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(diff, 0.0, 0.23, 0.0);

  return ret.getPose() * diff;*/
}

} //ns
