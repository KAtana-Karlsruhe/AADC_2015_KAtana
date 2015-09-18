// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \author  Philipp Hertweck <hertweck@fzi.de>
 * \date    2014-12-07
 *
 */
//----------------------------------------------------------------------
#include <vector>

#include "RoadBase.h"

using namespace std;

namespace katana
{

RoadBase::RoadBase(const RoadPatch::ConstPtr& road_patch, const Pose2d& pose, u_int32_t id, bool isVirtual)
  : m_detection_stamp(std::numeric_limits<double>::infinity())
  , m_matching_value(0.0)
{
  m_pose = pose;
  m_boundary = road_patch->getPatchBoundary();
  m_type = road_patch->getPatchType();
  m_endpose_container = road_patch->getEndPoseContainer();
  m_diversion_container = road_patch->getDiversionContainer();

  m_isVirtual = isVirtual;

  // save id
  m_id = id;

  // create diversion container for driving strips in world coordinates
  m_diversion_world_container.resize(getNumberOfDiversions(), RoadPatch::TrajectoryContainer());
  for(int diversion = 0 ; diversion < getNumberOfDiversions(); ++diversion) {
    for(int strip = 0; strip < getNumberOfDrivingStrips(diversion);++strip) {
      calculateWorldStrip(diversion, strip);
    }
  }

  // calculate world boundary coordinates
  calculateWorldBoundary();
}

void RoadBase::calculateWorldBoundary()
{
  // transform patch boundary points to world coordiantes
  for (std::size_t i = 0; i < m_boundary->size(); i++)
  {
    m_boundary_world[i] = m_pose * (*m_boundary)[i];
  }

  // transform patch end poses points to world coordiantes
  for (const Pose2d& i : *m_endpose_container)
  {
    m_endpose_world.push_back(m_pose * i);
  }
}

void RoadBase::calculateWorldStrip(DiversionId diversion, DrivingStripId strip)
{
  m_diversion_world_container[diversion].resize(getNumberOfDrivingStrips(diversion),RoadPatch::TrajectoryPtr());
  // create a copy of the chosen driving strip
  m_diversion_world_container[diversion][strip].reset(new Trajectory2d(*((*getDiversionContainer()).at(diversion).at(strip))));
  // transfrom this copy to world coordinates
  transformTrajectoryToWorld(*m_diversion_world_container[diversion][strip], m_pose);
}

void RoadBase::transformTrajectoryToWorld(Trajectory2d &trajectory, const Pose2d &pose)
{
  std::for_each(trajectory.begin(), trajectory.end(), [&](ExtendedPose2d& i){i.pose() = pose * i.pose();});
}

} //ns
