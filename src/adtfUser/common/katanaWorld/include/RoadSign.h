// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Philipp Hertweck <hertweck@fzi.de>
 * \date    2012-02-06
 *
 */
//----------------------------------------------------------------------

#ifndef _KATANA_WORLD_ROAD_SIGN_H
#define _KATANA_WORLD_ROAD_SIGN_H

#include <oadrive_core/Pose.h>
#include "katanaCommon/katanaCommon.h"

namespace katana
{
/**
 * @brief Represents a road sign
 */
class RoadSign
{

public:

  //! Convenience shared pointer
  typedef std::shared_ptr<RoadSign> Ptr;
  typedef std::shared_ptr<const RoadSign> ConstPtr;

  //! Constructor
  RoadSign()              {}
  RoadSign(const oadrive::core::Pose2d& p, const TrafficSign sign, const float size, const float yaw, bool isUpsideDown)
    : m_pose(p)
    , m_sign(sign)
    , m_size(size)
    , m_yaw(yaw)
    , m_isUpsideDown(isUpsideDown)
  {
  }

  //! Destructor
  virtual ~RoadSign()     {}

  //! Read Access
  const oadrive::core::Pose2d& getPose() const                  { return m_pose; }
  const TrafficSign getSign() const		 { return m_sign; }
  const float getArea() const			 { return m_size; }
  const float getYaw() const       { return m_yaw; }
  bool isUpsideDown() const        { return m_isUpsideDown; }


protected:
  //! Nearest pose
  oadrive::core::Pose2d m_pose;

  //! Represented type of sign
  TrafficSign m_sign;

  //! Size of the marker
  float m_size;

  //! estimated yaw respective to camera
  float m_yaw;

  //! if "top left" corner of marker is not far enough above "bottom left" corner
  bool m_isUpsideDown;

private:

};

} // ns

#endif //_KATANA_WORLD_OBSTACLE_H
