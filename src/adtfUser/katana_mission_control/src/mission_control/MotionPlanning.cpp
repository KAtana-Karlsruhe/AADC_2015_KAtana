#include "MotionPlanning.h"

#include <limits.h>
#include <cmath>

#include <oadrive_core/Interpolator.h>
#include <oadrive_core/TrajectoryVelocityInterpolator.h>

#ifdef KATANA_MC_MOTION_PLANNER_DEBUG
#include <iostream>
#endif

using namespace oadrive::core;

namespace katana
{

void MotionPlanning::generateRoadDrivingTrajectory(const vector<RoadBase::ConstPtr>& road_bases)
{
  // delete old trajectory
  m_standard_trajectory.clear();

  // reset junction state
  m_isJunctionInStrip = false;

  if (road_bases.empty())
  {
    m_driven_distance_since_last_update = 0;
    return;
  }

  // ensure that resulting trajectory only containes one driving direction
  // either forward OR reverse
  bool is_forward = road_bases.front()->getDrivingStrip(m_driving_strip)->isForwardTrajectory();

  // join driving strips
  vector<RoadBase::ConstPtr>::const_iterator junction_it;
  for (vector<RoadBase::ConstPtr>::const_iterator it = road_bases.begin();it != road_bases.end();++it)
  {
    // check if same driving direction as first
    assert(is_forward == (*it)->getDrivingStrip(m_driving_strip)->isForwardTrajectory() && "is_forward == (*it)->getDrivingStrip(m_driving_strip)->isForwardTrajectory()");
    if((*it)->getPatchType() == PatchType::JUNCTION){
      const RoadJunction::ConstPtr r_ptr_junction = std::static_pointer_cast<const RoadJunction>((*it));
      // choose appropriate driving strip
      m_standard_trajectory.append(*(r_ptr_junction->getDrivingStrip(m_driving_strip)));
      junction_it = it;
      m_isJunctionInStrip = true;
    } else {
      // choose appropriate driving strip
      m_standard_trajectory.append(*((*it)->getDrivingStrip(m_driving_strip)));
    }
  }

  m_standard_trajectory.isForwardTrajectory() = is_forward;

  prepareTrajectory(m_standard_trajectory);


  // Add Points from current pose to first Point (if needed)
  //!@todo
  //Trajectory2d extendedTrajectory = createTrajectoryToRoad(road_bases, trajectory);


#ifdef KATANA_MC_MOTION_PLANNER_DEBUG
  std::cout <<"MotionPlanning [updateDrivingStripFromPatches] - update driving strip: " <<std::endl;
  std::for_each(m_standard_trajectory.begin(), m_standard_trajectory.end(), [&](ExtendedPose2d& p){std::cout << p;});
#endif

  // reset driven distance counter after a new driving strip has been generated
  m_driven_distance_since_last_update = 0;
}

void MotionPlanning::prepareTrajectory(oadrive::core::Trajectory2d& trajectory) const
{
   // Try to remove overlapping points/points violating vehicle kinematics
  trajectory.shortcut();


  // BSpline smoothing
  Interpolator::smoothBSpline(trajectory, 3);

  if(trajectory.size() == 2) {
    for(u_int8_t i = 0; i < 3; ++i) {
      Interpolator::interpolateLinear(trajectory, 0.5);
    }
  }

  if(trajectory.size() > 2) {
    // Calculate additional trajectory data
    trajectory.calculateOrientations();
  }
  trajectory.calculateCurvature();

  //!@todo Write constant velocity on every extended pose
  double desired_speed = trajectory.isForwardTrajectory() ? m_default_speed : -m_default_speed;
  TrajectoryVelocityInterpolator::constantVelocityMotion(trajectory, desired_speed);
  trajectory.velocityAvailable() = true;
}

void MotionPlanning::setVelocityToDriveDistance(Trajectory2d &trajectory, size_t start_at, double desired_distance, double desired_speed) const
{
  assert(start_at < trajectory.size() && "start_at < trajectory.size()");

  // set to zero
  TrajectoryVelocityInterpolator::constantVelocityMotion(trajectory, 0.0);

  // find pose index when vehicle should stop
  size_t index_end = start_at + 1;

  for( ; index_end < trajectory.size(); index_end++)
  {
    if (trajectory.lengthBetween(start_at, index_end) > desired_distance)
      break;
  }


  if (index_end == start_at || index_end == trajectory.size()) // quite useless, do not drive
  {
    return;
  }

  //DEBUG
  std::cout <<"Writing speed to trajectory from " << 0 <<" to " <<index_end << " size of trajectory: " <<trajectory.size() <<std::endl;

  for (size_t i = 0; i < index_end; i++)
  {
    trajectory[i].setVelocity(desired_speed);
  }
  trajectory.velocityAvailable() = true;
}

//! create reverse trajectory from given trajectory
Trajectory2d MotionPlanning::createReverseTrajectory(const oadrive::core::Trajectory2d& trajectory) const
{
  Trajectory2d reverse;

  reverse = trajectory;
  std::reverse(reverse.begin(), reverse.end());

  reverse.isForwardTrajectory() = false;

  if(reverse.size() > 2) {
    // Calculate additional trajectory data
    reverse.calculateOrientations();
  }
  reverse.calculateCurvature();

  // reverse velocity
  if (reverse.velocityAvailable())
  {
    std::for_each(reverse.begin(), reverse.end(), [&](ExtendedPose2d& p) -> void {p.setVelocity(-p.getVelocity());});
  }

  return reverse;
}

} //ns
