// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-10
 *
 */
//----------------------------------------------------------------------

#ifndef _KATANA_WORLD_PINCONVERSIONS_H
#define _KATANA_WORLD_PINCONVERSIONS_H

#include "katanaCommon/katanaCommon.h"

#include "Obstacle.h"
#include "RoadSign.h"
#include <oadrive_core/Pose.h>

using namespace oadrive::core;

namespace katana
{

//////////////////////////////////////////////////////////
//! structs to write data to mediasamples
struct sPose    // C-Style struct
{
  _position_type x;
  _position_type y;
  _angle_type theta;

  sPose()
  {
  }

  sPose(const Pose2d& p)
  {
    fromPose(p);
  }

  void fromPose(const Pose2d& p)
  {
    x = p.translation().x();
    y = p.translation().y();
    theta = PoseTraits<Pose2d>::yaw(p);
  }
  // tmp function while mission control uses 1m coordinate system
  void fromPoseScaled(const Pose2d& p)
  {
    x = p.translation().x() * COORDINATE_SCALE_FACTOR_FROM_M;
    y = p.translation().y() * COORDINATE_SCALE_FACTOR_FROM_M;
    theta = PoseTraits<Pose2d>::yaw(p);
  }

  Pose2d toPose2d() const
  {
    Pose2d p;
    PoseTraits<Pose2d>::fromPositionAndOrientationRPY(p, x, y, theta);
    return p;
  }

  // tmp function while mission control uses 1m coordinate system
  Pose2d toPose2dScaled() const
  {
    Pose2d p;
    PoseTraits<Pose2d>::fromPositionAndOrientationRPY(p, x * COORDINATE_SCALE_FACTOR_TO_M, y * COORDINATE_SCALE_FACTOR_TO_M, theta);
    return p;
  }
};

struct sPatch    // C-Style struct
{
  sPose sp;
  int32_t patch_type;
  u_int32_t id;
  double match_value;
};

/**
 * @brief The sStatus struct
 * transmit state of mission control to perception
 */
struct sStatus
{
  MissionControlStatus status;
};

struct sObstacle // C-Style struct
{
  sPose sp;
  _stamp_type stamp;
  ObstacleSource source;
  float bounding_x;
  float bounding_y;

  void fromObstacle(const Obstacle& o)
  {
    sp.fromPoseScaled(o.getPose());
    bounding_x = o.getBoundingBox().first * COORDINATE_SCALE_FACTOR_FROM_M;
    bounding_y = o.getBoundingBox().second * COORDINATE_SCALE_FACTOR_FROM_M;
    stamp = o.getStamp();
    source = o.getSource();
  }

  Obstacle toObstacle() const
  {
    return Obstacle(sp.toPose2dScaled(), Obstacle::BoundingBox(bounding_x * COORDINATE_SCALE_FACTOR_TO_M, bounding_y * COORDINATE_SCALE_FACTOR_TO_M), stamp, source);
  }
};

struct sRoadSign
{
  sPose sp;
  TrafficSign sign;
  float size;
  float yaw;
  // true if sign is found in frame, false if sign disappeared
  bool signFound;

  bool isUpsideDown;

  void fromSign(const RoadSign& s)
  {
    sp.fromPoseScaled(s.getPose());
    sign = s.getSign();
    size = s.getArea();
    yaw = s.getYaw();
    isUpsideDown = s.isUpsideDown();
  }

  RoadSign toRoadSign() const
  {
    return RoadSign(sp.toPose2dScaled(), sign, size, yaw, isUpsideDown);
  }
};


} // ns

#endif //_KATANA_WORLD_PINCONVERSIONS_H
