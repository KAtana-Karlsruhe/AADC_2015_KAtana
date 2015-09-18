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

#ifndef _KATANA_WORLD_OBSTACLE_H
#define _KATANA_WORLD_OBSTACLE_H

#include "katanaCommon/katanaCommon.h"
#include <oadrive_core/Pose.h>
#include <oadrive_core/ExtendedPose2d.h>

namespace katana
{
/**
 * @brief Represents another road user or an obstacle
 */
class Obstacle
{
public:

  //! Convenience shared pointer
  typedef std::shared_ptr<Obstacle> Ptr;
  typedef std::shared_ptr<const Obstacle> ConstPtr;

  //! Bounding box
  typedef std::pair<katana::_position_type, katana::_position_type> BoundingBox;

  //!Quadrangle
  typedef std::array<oadrive::core::Position2d, 4> Quadrangle;

  //! Constructor
  Obstacle()              {}
  Obstacle(const oadrive::core::Pose2d& p, const BoundingBox& bounding_box, _stamp_type stamp, ObstacleSource source)
    : m_pose(p)
    , m_bounding_box(bounding_box)
    , m_stamp(stamp)
    , m_source(source)
  {
    Obstacle::calculateQuadrangle(p, bounding_box);
  }

  //! Destructor
  virtual ~Obstacle()     {}

  //! Read Access
  const oadrive::core::Pose2d& getPose() const                  { return m_pose; }
  const BoundingBox& getBoundingBox() const    { return m_bounding_box; }
  // time since start in 10^-6 seconds
  _stamp_type getStamp() const                 { return m_stamp; }
  ObstacleSource getSource() const	       { return m_source; }
  Quadrangle getQuadrangle() const             { return m_quadrangle; }
  
  _position_type getDistanceToObstacle(const oadrive::core::Pose2d& p)	{ return (p.translation() - m_pose.translation()).norm(); }

  //! Update values of old Obstacle if
  void updateValues(Ptr newObstacle)		{ 
    m_pose = newObstacle->getPose();
    m_bounding_box = newObstacle->getBoundingBox();
    m_source = newObstacle->getSource();
  }

protected:
  //! Center of the obstacle
  oadrive::core::Pose2d m_pose;

  //! Bounding box rectangle dimensions, x = first (depth), y = second (width)
  BoundingBox m_bounding_box;

  //! Time stamp
  _stamp_type m_stamp;
  
  //! Obstacle source
  ObstacleSource m_source;

  //! Quadrangle of the obstacle
  Quadrangle m_quadrangle;

private:
  //! Calculate Quadrangle of the Obstacle
  void calculateQuadrangle(oadrive::core::Pose2d p, BoundingBox bounding_box);

};

} // ns

#endif //_KATANA_WORLD_OBSTACLE_H
