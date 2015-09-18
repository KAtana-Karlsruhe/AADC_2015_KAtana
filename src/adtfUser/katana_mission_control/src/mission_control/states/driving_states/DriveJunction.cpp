#include "DriveJunction.h"
#include <helperClasses/HelperFunctions.h>
#include <limits>

#include <oadrive_core/TrajectoryVelocityInterpolator.h>

namespace katana {

void DriveJunction::onActivation(u_int32_t previous)
{
  #ifdef KATANA_MC_STATE_DEBUG
    std::cout <<"MissionControl - DrivingState: JUNCTION" <<std::endl;
  #endif

  // reset found junction
  //m_lt_found_junction = false;
  //m_notified_lanetracker = false;
  m_junction_state = SIGN_FOUND;

  m_junctionDetectionPose = m_system->getPositionController()->getCurrentVehiclePose();

  m_lastPoseOfJunctionSet = false;

  m_ignoring_obstacles = false;

  m_stopline_wait_counter = 0;
  m_waited_at_stopline = false;

  m_after_junction_found = false;
  m_notified_lanetracker = false;

  m_obstacle_stopline_counter = m_system->getConfig()->getDouble(CONF_WAIT_TIME_JUNCTION);

  // default speed
  m_system->getMotionPlanning()->defaultSpeed() = 0.2;

  // Let vehicle brake to search for junction
  oadrive::core::TrajectoryVelocityInterpolator::constantVelocityMotion(m_system->getPositionController()->getTrajectory(), 0.0);

  // blink
  Action nextDirection = m_system->getManeuver()->getCurrentManeuver();
  if(nextDirection == Action::LEFT) {
    m_system->getLightController()->setLight(LightName::TURN_LEFT, true);
  } else if(nextDirection == Action::RIGHT) {
    m_system->getLightController()->setLight(LightName::TURN_RIGHT, true);
  }

  //Trigger LT for junction
  callLanetrackerJunction();
}

bool DriveJunction::onStateLeave(u_int32_t next_state)
{
  // finished -> advance maneuver
  if (!m_system->getManeuver()->increaseManeuver())
  {
    m_change_state((u_int32_t)DrivingState::DRIVE_FINISHED);
    return false;
  }

  return true;
}


void DriveJunction::timerUpdate()
{
  // check if vehicle is physically blocked, e.g. after arduino crash
  m_system->timerCheckIfBlocked();

  // to avoid crashing into other cars, wait 5 seconds, then ignore all obstacles
  if (!m_ignoring_obstacles && m_system->timerCheckIfObstacleOnTrajectory())
  {
    if (m_system->getBlockedByObstacleCounter() > (u_int32_t)m_system->getConfig()->getDouble(CONF_OBSTACLE_WAITING_DURATION))
    {
      m_system->resetBlockedByObstacleCounter();


      m_ignoring_obstacles = true;
      m_system->getWorld()->emptyObstacles();
      m_system->getPositionController()->go(StopReason::OBSTACLE);
    }
  }

  // if driving over found junction, update trajectory
  if (m_junction_state == JUNCTION_FOUND)
  {
    // check if at stop line
    if (!m_waited_at_stopline && m_system->getPositionController()->hasReachedPosition(REACHED_POINT::RP_STOPLINE))
    {
      #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
	if(!m_waited_at_stopline)
	    std::cout <<"[DriveJunction] Waiting at stopline" <<std::endl;
      #endif

      // set timer counter
      m_stopline_wait_counter = STOP_LINE_WAITING_TIME;
      m_waited_at_stopline = true;
      m_system->getPositionController()->stop(StopReason::JUNCTION);
    }

    if (m_stopline_wait_counter > 0)
    {
      if (m_stopline_wait_counter == 1)
      {
        m_system->getWorld()->emptyObstacles();
        m_system->getPositionController()->go(StopReason::JUNCTION);
        m_system->getPositionController()->go(StopReason::OBSTACLE);
	m_system->resetBlockedByObstacleCounter();
      }
      Obstacle::Ptr obs = m_system->getCollisionHandler()->collision(katana::CollisionHandler::BOX_JUNCTION_FRONT_OF_CAR, m_junction_pose);
      if(m_system->timerCheckIfObstacleInFront(obs) && m_obstacle_stopline_counter > 0) {
       #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
	  if(m_obstacle_stopline_counter == m_system->getConfig()->getDouble(CONF_WAIT_TIME_JUNCTION))
	    std::cout << "[DriveJunction] Obstacle on Junction. WAITING" << std::endl;
       #endif

	--m_obstacle_stopline_counter;
      }
      else
      {
        --m_stopline_wait_counter;
      }
    }

    // update trajectory
    m_system->timerUpdateTrajectory();
  }

  if(m_lastPoseOfJunctionSet)
  {
    if(m_system->getPositionController()->hasReachedPosition(REACHED_POINT::RP_JUNCTION))
    {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      std::cout <<"[StateDriving] End of junction reached" <<std::endl;
#endif

      // stop blinking
      m_system->getLightController()->setLight(TURN_LEFT, false);
      m_system->getLightController()->setLight(TURN_RIGHT, false);

      // Lanetracker found newPatch (this should be the verification of the junction) -> go on driving
      m_change_state((u_int32_t)DrivingState::DRIVE_NORMAL);
      return;

    }
  }
}

void DriveJunction::newTrafficSign(RoadSign::Ptr sign, bool signAppeared)
{
  katana::DriveBase::newTrafficSign(sign, signAppeared);

  if(m_system->isJunctionSign(sign) && !signAppeared) {
    #ifdef KATANA_MC_JUNCTION_LOGIC_DEBUG
      std::cout << "[DriveJunction] Sign disappeared." << std::endl;
    #endif

//    m_system->getPositionController()->stop(JUNCTION);

    m_seeing_sign = false;
  }
}


void DriveJunction::newPatch(size_t num, PerceptionState perception_state, u_int32_t job_id)
{
  #ifdef KATANA_MC_JUNCTION_LOGIC_DEBUG
    std::cout << "[DriveJunction] received " << num << " patches with state " << perception_state << std::endl;
    std::cout << "[DriveJunction] state " << m_junction_state << std::endl;
  #endif

  switch (m_junction_state) {
    // sign found, but lanetracker not notified
    case SIGN_FOUND : {
      if(perception_state == JUNCTION_AHEAD) {
	  assert(false && "[DriveJunction] Junction found but not triggered.");
      } else {
	  //retrigger Lanetracker
	#ifdef KATANA_MC_JUNCTION_LOGIC_DEBUG
	  std::cout << "[DriveJunction] Retrigger LT." << std::endl;
	#endif
	callLanetrackerJunction();
	m_junction_state = LT_JUNCTION_SEARCH;
      }
      break;
    }
    // lanetracker notified to find junction
    case LT_JUNCTION_SEARCH : {
      // answer of right request
      if((perception_state == JUNCTION_AHEAD || perception_state == JUNCTION_AHEAD_AGAIN) && num > 0) {
	#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
	  std::cout << "[DriveJunction] LT found junction. Starting to drive over." << std::endl;
	#endif
	m_junction_state = JUNCTION_FOUND;
	junctionFound();
      }
      // LT was notified but junction was not detected
      else if((perception_state == JUNCTION_AHEAD || perception_state == JUNCTION_AHEAD_AGAIN) && num == 0) {
	#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
	  std::cout << "[DriveJunction] LT found NO junction." << std::endl;
	#endif

	// Check if we think we are over the junction
	if(missedJunction()) {
	  #ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
	    std::cout << "[DriveJunction] Guess we missed the junction. Trying to drive without detected junction." << std::endl;
	  #endif
	  m_junction_state = JUNCTION_FOUND;
	  insertVirtualJunctionToWorld();
	  junctionFound();
	}
	// Check if there is a trajectory to drive
	else if(m_system->getPositionController()->hasReached()) {
	  #ifdef KATANA_MC_JUNCTION_LOGIC_DEBUG
	    std::cout << "[DriveJunction] No trajectory to drive on." << std::endl;
	    std::cout << "[DriveJunction] Trigger lanetracker to search for new patches in front of junction." << std::endl;
	  #endif
	  // Let the LT search for new patches
	  m_junction_state = LT_STRAIGHT_SEARCH;
	  m_system->callLanetrackerInitial(0, oadrive::vision::PatchesToLookFor::STRAIGHTS, 0.2);

	} else {
	  driveForward();
	  callLanetrackerJunction();
	}
      }
      else {
	callLanetrackerJunction();
      }
      break;
    }
    case LT_STRAIGHT_SEARCH : {
      if(num > 0) {
	#ifdef KATANA_MC_JUNCTION_LOGIC_DEBUG
	  std::cout << "[DriveJunction] Found new straight patches in front of junction." << std::endl;
	#endif
	// new patches already added to world
	m_system->updatePlannedMotion();
	driveForward();
	// retrigger Lanetracker
	callLanetrackerJunction();

      } else {
	#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
	  std::cout << "[DriveJunction] Junction not found and no other patches found. Driving with virtual junction." << std::endl;
	#endif

	m_junction_state = JUNCTION_FOUND;
	insertVirtualJunctionToWorld();
	junctionFound();
      }
      break;
    }
    case JUNCTION_FOUND : {
      // only re-notify lanetracker
      const World::RoadPatchContainerConst& next_patches = m_system->getWorld()->getNextPatches();
      if(num > 0 || m_after_junction_found) {
          m_after_junction_found = true;
  // if we already found a match in after junction, only accept good matches from here on
  m_system->getLanetrackerJobManager()->submitJob(next_patches, NORMAL, 0, oadrive::vision::PatchesToLookFor::STRAIGHTS, -1.0);
      } else {
  // search with normal tolerance (to avoid patch shifting) but accept low match values
	m_system->getLanetrackerJobManager()->submitJob(next_patches, AFTER_JUNCTION, 0, oadrive::vision::PatchesToLookFor::STRAIGHTS, 0.1);
      }
      break;
    }

    default : assert(false && "[DriveJunction] Unknown junction state.");
  }
}

bool DriveJunction::missedJunction()
{
  if(/*!m_seeing_sign ||*/ (m_junctionDetectionPose.translation() - m_system->getPositionController()->getCurrentVehiclePose().translation()).norm()
                              > m_system->getConfig()->getDouble(CONF_JUNCTION_MISSED_DISTANCE)) {
    return true;
  }

  return false;
}

void DriveJunction::insertVirtualJunctionToWorld()
{
  RoadBase::Ptr guess = getJunctionGuess();

  m_system->getWorld()->addRoadBase(guess, m_system->getPositionController()->getOverallDrivenDistance());

  // set direction
  Action nextDirection = m_system->getManeuver()->getCurrentManeuver();
  if(nextDirection == Action::LEFT || nextDirection == Action::RIGHT || nextDirection == Action::STRAIGHT) {
    m_system->getWorld()->setDirection(nextDirection);
  }

  // set the saved traffic sign to the junction
  //m_system->getWorld()->setJunctionRightOfWay();
}

void DriveJunction::junctionFound()
{
  // clear patches in world, leave only the junction
  m_system->getWorld()->deleteAllPatchesButJunction();

  // create straight patches at end of junction
  const World::RoadPatchContainerConst& only_junction = m_system->getWorld()->getNextPatches();

  m_junction_pose = only_junction.front()->getAnchorPose();
  const Position2d stop_line = m_system->getPositionController()->getStopLineReachedPose(m_junction_pose);
  m_system->getPositionController()->setPositionToCheck(REACHED_POINT::RP_STOPLINE, stop_line, 0.03);

  driveOverJunction(only_junction);
}

void DriveJunction::driveOverJunction(const World::RoadPatchContainerConst& only_junction)
{
  m_system->getPositionController()->go(StopReason::JUNCTION_NOT_FOUND);

  const World::RoadPatchContainer virtualPatches = HelperFunctions::getVirtualPatches(m_system, only_junction.front());

  // even if lanetracker does not see the virtually added patches, already add them to world
  for (const RoadBase::Ptr& rb : virtualPatches)
  {
    m_system->getWorld()->addRoadBase(rb, m_system->getPositionController()->getOverallDrivenDistance());
  }

  // now a driving strip over the junction can be built
  const World::RoadPatchContainerConst& next_patches = m_system->getWorld()->getNextPatches();

  // The vehicle can continue to drive with constant velocity
  // update to standard trajectory
  m_system->updatePlannedMotion(next_patches);

  // save target pose when end of junction is reached
  assert(next_patches.front()->isJunction() == true && "next_patches.front()->isJunction() == true");
  Position2d nearest_on_trajectory;
  if (next_patches.size() > 1)
  {
    const Position2d junction_end = next_patches[1]->getDrivingStrip(0)->front().getPosition();
    nearest_on_trajectory = m_system->getPositionController()->getNearestPoseOnCurrentTrajectory(junction_end).translation();
  }
  else
  {
    nearest_on_trajectory = m_system->getPositionController()->getTrajectory().back().getPose().translation();
  }

  // let position controller check
  m_system->getPositionController()->setPositionToCheck(REACHED_POINT::RP_JUNCTION, nearest_on_trajectory);
  m_lastPoseOfJunctionSet = true;

  // notify lanetracker
  m_system->getLanetrackerJobManager()->submitJob(next_patches, AFTER_JUNCTION, 0, oadrive::vision::PatchesToLookFor::STRAIGHTS, -1.0);
}

RoadBase::Ptr DriveJunction::getJunctionGuess() {
  Pose2d estimatedJunctionStart;

  u_int32_t lastId = m_system->getWorld()->getLastPatchId();

  estimatedJunctionStart = getStartPoseNextToCar();

  return m_system->getWorld()->instantiateRoadJunction(PatchType::JUNCTION, estimatedJunctionStart, lastId+1);
}


Pose2d DriveJunction::getStartPoseNextToCar()
{
  //Pose2d lastPatch = m_system->getPatchUnderCar()->getAnchorPose();

  //double distance = std::numeric_limits<double>::max();

  // Calculate front of car
  Pose2d delta;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(delta, 1.05, 0.23, 0);
  Pose2d car_front = m_system->getPositionController()->getCurrentVehiclePose() * delta;

  // create vector
  HelperFunctions::PoseVector v;

  const World::RoadPatchContainerConst nextPatches = m_system->getWorld()->getNextPatches();
  if (!nextPatches.empty())
  {
    for(const RoadBase::ConstPtr& patch : nextPatches)
    {
      v.push_back(patch->getAnchorPose());
    }
    v.push_back(nextPatches.back()->getEndPose());

    Pose2d projected_pose;
    // this maps car_front onto v, if returns false, projection is behind end of v

    std::cout <<"PROJECTION: Current vehicle pose: " <<m_system->getPositionController()->getCurrentVehiclePose() <<" car front: " <<car_front <<std::endl;

    if (!HelperFunctions::projectPoseOnTrajectory(car_front.translation(), v, projected_pose))
      return car_front;

    return projected_pose;
  }

  return car_front;
}

void DriveJunction::callLanetrackerJunction()
{
  if(!m_system->getLanetrackerJobManager()->isLTWorking()) {
    std::vector<katana::RoadBase::ConstPtr> vector(1, getJunctionGuess());
    if (!m_notified_lanetracker)
    {
      m_notified_lanetracker = true;
      m_system->getLanetrackerJobManager()->submitJob(vector, JUNCTION_AHEAD, 0, 0, -1.0);
      m_system->getPositionController()->stop(StopReason::JUNCTION_NOT_FOUND);
    }
    else
    {
      m_system->getLanetrackerJobManager()->submitJob(vector, JUNCTION_AHEAD_AGAIN, 0, 0, -1.0);
      m_system->getPositionController()->go(StopReason::JUNCTION_NOT_FOUND);
    }
    m_junction_state = LT_JUNCTION_SEARCH;
  }
}

void DriveJunction::driveForward()
{
  #ifdef KATANA_MC_JUNCTION_LOGIC_DEBUG
    std::cout << "[DriveJunction] Driving forward." << std::endl;
  #endif

  m_system->getPositionController()->go(JUNCTION);

  m_system->getMotionPlanning()->setVelocityToDriveDistance(
        m_system->getPositionController()->getTrajectory(),
        m_system->getPositionController()->getProjectedPoseIndex(),
        0.2,
        m_system->getMotionPlanning()->defaultSpeed());
}


} // end of ns
