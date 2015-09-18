// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Raphael Frisch <frisch@fzi.de>
 * \date    2015-03-05
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_COLLISION_HANDLER_H
#define _MISSION_CONTROL_COLLISION_HANDLER_H

#include <thread>
#include <chrono>
#include <mutex>

#include "katanaCommon/katanaCommon.h"

#include "mission_control/world.h"
#include "Obstacle.h"

#include <queue>

namespace katana
{

/**
 * @brief The Collision Handler class
 * Ensures that does not collide with any obstacle
 */
class CollisionHandler
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<CollisionHandler> Ptr;
  typedef std::array<oadrive::core::Position2d, 4> Quadrangle;

  //! Constructor
  CollisionHandler(World::Ptr world)
    : m_world(world)
  {

  }

  //! No copying
  CollisionHandler() = delete;
  CollisionHandler(const CollisionHandler& rs) = delete;
  CollisionHandler& operator=(const CollisionHandler& rhs) = delete;

  //! Destructor
  virtual ~CollisionHandler()  {}

//  //!Update collision calculation
//  void updateCollisionCalculation();

//  //! Read flags
//  bool mustStop() const { return m_vehicle_must_stop; }
//  bool mustChangeDrivingStrip() const { return m_change_driving_strip; }
//  Obstacle::Ptr getRelevantObstacle() {return relevantObstacle; }

  Obstacle::Ptr collision(const Quadrangle& q, const Pose2d& p) const;
  Obstacle::Ptr collisionDriving(const Pose2d& p, float steeringAngle) const;

  Obstacle::Ptr collisionTrajectory(const oadrive::core::Trajectory2d& trajectory,
                                    std::size_t begin_index, const double& diff_distance,
                                    std::size_t number_boxes, const Quadrangle& box) const;


  static const Quadrangle BOX_IN_FRONT_OF_CAR;
  static const Quadrangle BOX_LONG_IN_FRONT_OF_CAR;
  static const Quadrangle BOX_PARK_LEFT_SIDE_OF_CAR;
  static const Quadrangle BOX_PARK_RIGHT_SIDE_OF_CAR;
  static const Quadrangle BOX_PARK_BEHIND_CAR;
  static const Quadrangle BOX_PARK_FRONT_OF_CAR;
  static const Quadrangle BOX_PARK_SMALL_FRONT_OF_CAR;
  static const Quadrangle BOX_PARK_SMALL_BEHIND_OF_CAR;
  static const Quadrangle BOX_PARK_PULL_OUT_BEHIND;
  static const Quadrangle BOX_JUNCTION_FRONT_OF_CAR;

  static const Quadrangle BOX_CAR_ON_TRAJECTORY;


  static constexpr double STEERING_TO_ANGLE_FACTOR = 0.0610855; //0.35 * 0.017453;
private:
//  //! Checks if there is an Obstacle in front of the car by using elementInBox method
//  bool obstacleInFrontOfCar(Quadrangle obstacle) const;

//  //! Checks if the car if there are obstacles on the other driving strip by using elementInBox method
//  bool canChangeDrivingStrip(World::ObstacleContainer &cont) const;

  bool obstacleInBox(const Obstacle& obs, const Quadrangle& q) const;

  bool overlapping(const Quadrangle& q1, const Quadrangle& q2) const;

  bool isWithinOrOnLineOfQuadrangle(const Position2d& point, const Quadrangle& q) const;

  //! Convert given Quadrangle to world coordinates
  void createWorldCoordinateBox(const oadrive::core::Pose2d& p, const Quadrangle& box, Quadrangle& box_world) const;

  //! Flags
  Obstacle::Ptr relevantObstacle;
  bool m_vehicle_must_stop; //!< if set to true, the positionController must stop the vehicle

  bool m_change_driving_strip; //!< if set to true, the motionPlanning must change the driving strip

  //! World pointer
  World::Ptr m_world;

  //! Tmp. world coordinate quadrangle
  mutable Quadrangle m_tmp_world_quadrangle;

};


} // ns

#endif //_MISSION_CONTROL_COLLISION_HANDLER_H
