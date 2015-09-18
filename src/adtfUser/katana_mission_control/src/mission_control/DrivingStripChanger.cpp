// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-19
 *
 */
//----------------------------------------------------------------------

#include "DrivingStripChanger.h"

#define KATANA_MC_DRIVING_CHANGE_DEBUG

#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
#include <iostream>
#endif

using namespace oadrive::core;

namespace katana
{

void DrivingStripChanger::changeLane(bool change_back_automaticly)
{
#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
  std::cout <<"[DrivingStripChanger] Initiating lane change." <<std::endl;
#endif
  //Driving slower during overtaking
  m_system->getMotionPlanning()->defaultSpeed() = 0.15;

  // Currently in overtaking mode? Disable parking assistant
  if (m_overtaking)
  {
    m_system->getWorld()->disableSearchForParkingSpot();
    m_parking_assistant.reset();
  }

  // determine if will be overtaking
  m_overtaking = m_system->getMotionPlanning()->drivingStrip() == DEFAULT_DRIVING_STRIP && change_back_automaticly;

  // take distance
  m_driven_distance_checked = m_system->getPositionController()->getOverallDrivenDistance();

  // toggle driving strip
  const u_int8_t strip = m_system->getMotionPlanning()->drivingStrip() == 0 ? 1 : 0;
#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
  std::cout <<"[DrivingStripChanger] Setting driving strip to: " << (u_int32_t)strip << " Driven distance is: " << m_system->getPositionController()->getOverallDrivenDistance() << std::endl;
#endif
  m_system->getMotionPlanning()->drivingStrip() = strip;

  // turn signal
  if (strip == 0)
  {
    m_system->getLightController()->setLight(LightName::TURN_RIGHT, true);
  }
  else
  {
    m_system->getLightController()->setLight(LightName::TURN_LEFT, true);
  }

  // re-build trajectory
  m_system->updatePlannedMotion();

  // now vehicle is chaning lane
  m_changing_lane = true;

  //set reached point ~1,5 m in front of vehicle to turn off turn signals
  Pose2d diff;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(diff, 0.5, 0.0, 0.0);
  const Pose2d target = m_system->getPositionController()->getCurrentVehiclePose() * diff;
  const Pose2d on_trajectory = m_system->getPositionController()->getNearestPoseOnCurrentTrajectory(target.translation());
  m_system->getPositionController()->setPositionToCheck(REACHED_POINT::RP_OVERTAKING, on_trajectory.translation(), 0.03);

  if (m_overtaking)   //< vehicle needs to switch back as soon as possible
  {
#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
    std::cout <<"[DrivingStripChanger] Enabling parking assistant to check for empty lane." <<std::endl;
#endif

    // turn on search for empty lane
    m_parking_assistant = m_system->getWorld()->searchForParkingSpot(ParkingSpace::PARALLEL, DEFAULT_DRIVING_STRIP == 0 ? RIGHT : LEFT, 10 );
    m_parking_assistant->setWaitForRear(false);
  }
}

void DrivingStripChanger::timerCheck()
{
  // only when currently overtaking check for empty lane
  if (m_overtaking)
  {
    if (m_system->getPositionController()->getOverallDrivenDistance() - DISTANCE_TO_CHECK_LANE > m_driven_distance_checked)
    {
      m_driven_distance_checked = m_system->getPositionController()->getOverallDrivenDistance();
      m_parking_assistant->evaluateMeasurments(m_driven_distance_checked);

      double distance_dummy;
      if (m_parking_assistant->foundAppropriateSpot(distance_dummy))    //< other lane seems to be empty
      {
#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
        std::cout <<"[DrivingStripChanger] Other lane seems empty - change back..." <<std::endl;
#endif

        // simply change back
        changeLane();
      }
    }

  }

  // is lane changing finished? -> turn off turn signal
  if (m_changing_lane)
  {
    if (m_system->getPositionController()->hasReachedPosition(REACHED_POINT::RP_OVERTAKING))
    {
#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
      std::cout <<"[DrivingStripChanger] Lane changing finished" <<std::endl;
#endif

      // lane change finished
      m_changing_lane = false;

      //Driving faster in normal case
      m_system->getMotionPlanning()->defaultSpeed() = m_system->getConfig()->getDouble(DoubleParameter::CONF_STANDARD_VELOCITY);

      // turn off turn signal
      m_system->getLightController()->setLight(LightName::TURN_RIGHT, false);
      m_system->getLightController()->setLight(LightName::TURN_LEFT, false);
    }
  }

  // currently reversing?
  if (m_reversing)
  {
    if (m_system->getPositionController()->hasReached())
    {
#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
  std::cout <<"[DrivingStripChanger] REVERSING FINISHED" <<std::endl;
#endif
      m_reversing = false;

      changeLane(m_change_back_automaticly);
    }
  }

}

double DrivingStripChanger::needToReverse() const
{
  // obtain blocking obstacle
  Obstacle::ConstPtr obs = m_system->checkObstacleTrajectory();

  if (obs == nullptr)
    return -1.0;

  // calculate distance to front of vehicle
  Pose2d diff;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(diff, 0.45, 0.0, 0.0);

  const Pose2d obs_pose = (m_system->getPositionController()->getCurrentVehiclePose() * diff).inverse() * obs->getPose();

  std::cout <<obs_pose <<std::endl;

  double to_obstacle = obs_pose.translation().x() - m_system->getConfig()->getDouble(CONF_DISTANCE_NEEDED_FOR_LANE_CHANGE);

  if (obs_pose.translation().y() < 0.15)
    to_obstacle = 0.0;

  if (to_obstacle > 0.0)
    to_obstacle = 0.0;

  return to_obstacle;

}

void DrivingStripChanger::driveReverse(double dist, bool change_back_automaticly = false)
{
  assert(!m_reversing && "Already reversing!");

#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
  std::cout <<"[DrivingStripChanger] REVERSING" <<dist <<std::endl;
#endif

  // remember option
  m_change_back_automaticly = change_back_automaticly;

  // current trajectory as reverse...
  Trajectory2d reverse = m_system->getMotionPlanning()->createReverseTrajectory(m_system->getPositionController()->getTrajectory());

  // drive only dist
  m_system->getMotionPlanning()->setVelocityToDriveDistance(
        reverse,
        m_system->getPositionController()->getNearestIndexOnTrajectory(reverse, m_system->getPositionController()->getCurrentVehiclePose().translation()),
        dist,
        -0.15);

  // shorten trajectory
  Trajectory2d::iterator it;
  for (it = reverse.begin(); it != reverse.end(); it++)
  {
    if (it->getVelocity() == 0.0)
      break;
  }

  if (it != reverse.begin())
  {
    reverse.erase(it, reverse.end());
  }

  // set trajectory
#ifdef KATANA_MC_DRIVING_CHANGE_DEBUG
 std::cout <<"[DrivingStripChanger] REVERSING" <<std::endl;
#endif

  m_reversing = true;
  m_system->getPositionController()->setTrajectory(reverse);
}

} // ns
