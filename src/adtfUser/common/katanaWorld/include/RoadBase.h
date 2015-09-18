// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-07
 *
 */
//----------------------------------------------------------------------

#ifndef _KATANA_WORLD_ROADBASE_H
#define _KATANA_WORLD_ROADBASE_H

#include <vector>
#include <iostream>

#include "katanaCommon/katanaCommon.h"
#include "RoadPatch.h"
#include "Obstacle.h"

namespace katana
{

enum class DIVERSION : u_int8_t {
  UNDEFINED =0,
  RIGHT=1,
  STRAIGHT=2,
  LEFT=3
};

class RoadBase
{
public:
  //! Shared pointer shorthand
  typedef std::shared_ptr<RoadBase> Ptr;
  typedef std::shared_ptr<const RoadBase> ConstPtr;

  //! Patch Boundary in world coordinates
  typedef std::array<Position2d, (u_int32_t)RoadPatch::CORNER_POINT::CORNER_POINT_COUNT> PatchBoundaryPolygon;
  //! ID of the driving Strip
  typedef u_int8_t DrivingStripId;
  //! ID of the diversion
  typedef u_int8_t DiversionId;

  const RoadBase::DiversionId DEFAULT_DEVERSION = 0;

  //! Constructor - make RoadBase non-copyable
  RoadBase() = delete;
  RoadBase(const RoadBase& rb) = delete;
  RoadBase& operator =(const RoadBase& rb) = delete;
  //! Construct from template
  RoadBase(const RoadPatch::ConstPtr& road_patch, const Pose2d& pose, u_int32_t id, bool isVirtual = false);

  //! Destructor
  virtual ~RoadBase()    {}

  //! @see m_detection_stamp
  void setDetectionStamp(double detection_stamp)        { m_detection_stamp = detection_stamp; }
  double getDetectionStamp() const                      { return m_detection_stamp; }

  //! Is this a virtual patch, so patch type is sure
  bool isVirtual() const				{ return m_isVirtual; }
  void setVirtual() 					{ m_isVirtual = true; }

  //! Is this a normal patch or a junction? (has to be virtual)
  virtual bool isJunction() const     { return false; }

  //! GetID
  u_int32_t getId() const       { return m_id; }

  //! set the driving direction
  virtual void setDirection(Action direction)	{ assert(false && "setDirection called on Roadbase. This is maybe not what you want"); }
  virtual Action getDirection() const		{ assert(false && "getDirection called on Roadbase. This is maybe not what you want"); }

  //! Read access to patch-coordinate points
  virtual RoadPatch::DiversionConstContainerConstPtr getDiversionContainer() const	{ return m_diversion_container; }

  //! Get the PointArray of driving strip with given ID using default deversion
  virtual RoadPatch::TrajectoryPtr getDrivingStrip(DrivingStripId drivingStrip) const     { return m_diversion_world_container[DEFAULT_DEVERSION][drivingStrip]; }
  //! Get the PointArray of driving strip with given ID
  RoadPatch::TrajectoryPtr getDrivingStrip(DrivingStripId drivingStrip, DiversionId diversion) const	{ return m_diversion_world_container[diversion][drivingStrip]; }

  //! Read access
  const Pose2d& getAnchorPose() const                                         { return m_pose; }
  const RoadPatch::PoseContainer getEndPoseContainer() const                  { return m_endpose_world; }
  const Pose2d getEndPose() const                                             { return m_endpose_world.at(0); }
  PatchType getPatchType() const	                                      { return m_type; }
  const int getNumberOfDrivingStrips(DrivingStripId id)			      { return m_diversion_container->at((int32_t)id).size(); }
  const int getNumberOfDiversions() const				      { return m_diversion_container->size(); }

  //! Boundary Polygon
  const Position2d& getBoundingPoint(RoadPatch::CORNER_POINT which) const     { return m_boundary_world[(u_int8_t)which];}
  const PatchBoundaryPolygon& getBoundaryPolygon() const                      { return m_boundary_world; }

  //! Check if a given pose is located on a patch
  bool isPoseOnPatch(const Pose2d& pose) const                                { return isPointOnPatch(pose.translation()); }

  double distanceToAnchorPose(const Position2d& point) const    { return (point - m_pose.translation()).norm(); }

  //! Generate debug output
  virtual void printToConsole()	const     { std::cout << "Roadbase: " <<m_pose << " id " << m_id << std::endl; }


  //! Convert the given trajectory to world coordinates
  static void transformTrajectoryToWorld(Trajectory2d &trajectory, const Pose2d& pose);

  //! Matching value
  double& matchingValue()              { return m_matching_value; }
  const double& matchingValue() const  { return m_matching_value; }

protected:
  //! Anchor pose
  Pose2d m_pose;

  //! Type of this patch
  PatchType m_type;

  //! Endpose Container from Patch template (const - does not change)
  RoadPatch::EndPoseContainerConstPtr m_endpose_container;

  //! Endpose Container in world coordinates
  RoadPatch::PoseContainer m_endpose_world;

  //! Boundary of patch (patch coordinates)
  RoadPatch::PatchBoundaryConstPtr m_boundary;

  //! Boundary of patch (world coordinates -> calculated in constructor after pose is set)
  PatchBoundaryPolygon m_boundary_world;

  //! Diversion Container holding Driving strip container (does not change, pointer to points from template patch)
  RoadPatch::DiversionConstContainerConstPtr m_diversion_container;

  //! Driving strips of all diversion in world coordinates
  RoadPatch::DiversionContainer m_diversion_world_container;



private:
  //! Calculate world coordinates of specific driving strip and save to m_driving_strip_world
  void calculateWorldStrip(DrivingStripId strip);
  void calculateWorldStrip(DiversionId diversion, DrivingStripId strip);

  //! Calculate world coordinate bounding points -> call in constructor!
  void calculateWorldBoundary();

  //!
  bool isPointOnPatch(const Position2d& point) const;

  //! Unique ID of this patch
  u_int32_t m_id;

  //! Driven distance at moment of detection, used to delete old patches
  double m_detection_stamp;

  //! Match value of this particular patch
  double m_matching_value;

  //! Is this a virtual patch, so patch type is sure
  bool m_isVirtual;
};

} // ns

#endif //_KATANA_WORLD_ROADBASE_H
