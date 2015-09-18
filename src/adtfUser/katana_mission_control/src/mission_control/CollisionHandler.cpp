// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-06
 *
 */
//----------------------------------------------------------------------

#include "mission_control/CollisionHandler.h"

#ifdef KATANA_MC_COLLISION_HANDLER
#include <iostream>
#endif

namespace katana
{

//! Box in front of the car. Obstacles in this box stop the car
const CollisionHandler::Quadrangle CollisionHandler::BOX_IN_FRONT_OF_CAR = {
  Position2d(0.45, 0.15),
  Position2d(0.45, -0.15),
  Position2d(0.8, -0.15),
  Position2d(0.8, 0.15)};
const CollisionHandler::Quadrangle CollisionHandler::BOX_LONG_IN_FRONT_OF_CAR = {
  Position2d(0.09, 0.15),
  Position2d(0.09, -0.15),
  Position2d(0.9, -0.1),
  Position2d(0.9, 0.1)};

//! Boxes needed for localisation for the first pull-out
const CollisionHandler::Quadrangle CollisionHandler::BOX_PARK_RIGHT_SIDE_OF_CAR = {
  Position2d(-0.11, -0.15),
  Position2d(-0.11, -0.35),
  Position2d(0.48, -0.35),
  Position2d(0.48, -0.15)};
const CollisionHandler::Quadrangle CollisionHandler::BOX_PARK_LEFT_SIDE_OF_CAR = {
  Position2d(-0.11, 0.35),
  Position2d(-0.11, 0.15),
  Position2d(0.48, 0.15),
  Position2d(0.48, 0.35)};
const CollisionHandler::Quadrangle CollisionHandler::BOX_PARK_BEHIND_CAR = {
  Position2d(-0.41, 0.15),
  Position2d(-0.41, -0.15),
  Position2d(-0.11, -0.15),
  Position2d(-0.11, 0.15)};
const CollisionHandler::Quadrangle CollisionHandler::BOX_PARK_FRONT_OF_CAR = {
  Position2d(0.45, 0.15),
  Position2d(0.45, -0.15),
  Position2d(0.75, -0.15),
  Position2d(0.75, 0.15)};

//!Boxes needed to park in
const CollisionHandler::Quadrangle CollisionHandler::BOX_PARK_SMALL_FRONT_OF_CAR = {
  Position2d(0.48, 0.02),
  Position2d(0.48, -0.02),
  Position2d(0.58, -0.02),
  Position2d(0.58, 0.02)};
const CollisionHandler::Quadrangle CollisionHandler::BOX_PARK_SMALL_BEHIND_OF_CAR = {
  Position2d(-0.21, 0.02),
  Position2d(-0.21, -0.02),
  Position2d(-0.11, -0.02),
  Position2d(-0.11, 0.02)};
const CollisionHandler::Quadrangle CollisionHandler::BOX_PARK_PULL_OUT_BEHIND = {
  Position2d(-0.17, 0.02),
  Position2d(-0.17, -0.02),
  Position2d(-0.11, -0.02),
  Position2d(-0.11, 0.02)};

const CollisionHandler::Quadrangle CollisionHandler::BOX_CAR_ON_TRAJECTORY = {
  Position2d(0.0, 0.125),
  Position2d(0.0, -0.125),
  Position2d(0.35, -0.125),
  Position2d(0.35, 0.125)};

const CollisionHandler::Quadrangle CollisionHandler::BOX_JUNCTION_FRONT_OF_CAR = {
  Position2d(0, 0.5),
  Position2d(0, -0.5),
  Position2d(1, -0.5),
  Position2d(1, 0.5)};



Obstacle::Ptr CollisionHandler::collisionDriving(const Pose2d& p, float steeringAngle) const
{
  CollisionHandler::Quadrangle currBox;

  Pose2d rotatedBoxPose;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(rotatedBoxPose, 0.36, 0.0, steeringAngle * STEERING_TO_ANGLE_FACTOR);
  //TODO Rotate the Steering Angle around the front wheels

  const Pose2d transf = p * rotatedBoxPose;

  for(size_t i = 0; i < 4; i++)
  {
    currBox[i] = (transf * BOX_LONG_IN_FRONT_OF_CAR[i]).eval();
  }

  #ifdef KATANA_MC_COLLISION_HANDLER
    std::cout << "MC - CollisionHandler: collisionDriving: corner left near of box x= " << currBox[0].x() << " y= " << currBox[0].y() << std::endl;
    std::cout << "MC - CollisionHandler: collisionDriving: SteeringAngle for Box: " << steeringAngle * STEERING_TO_ANGLE_FACTOR << std::endl;
  #endif



  const katana::World::ObstacleContainer& cont =  m_world->getObstacleContainerPtr();

  #ifdef KATANA_MC_COLLISION_HANDLER
    std::cout << "MC - CollisionHandler: collisionDriving: Obstacles in world: " << cont.size() << std::endl;
  #endif
  for( size_t i = 0; i < cont.size();i++)
  {
    if(obstacleInBox(*cont[i], currBox))
    {
      #ifdef KATANA_MC_COLLISION_HANDLER
        std::cout << "MC - CollisionHandler: collisionDriving: Obstacle x= " << cont[i]->getPose().translation()[0] << " , y= " << cont[i]->getPose().translation()[1] << std::endl;
      #endif
      return cont[i];
    }
  }
  return Obstacle::Ptr();

}




Obstacle::Ptr CollisionHandler::collision(const Quadrangle& q, const Pose2d& p) const
{
  // world coordinates
  createWorldCoordinateBox(p, q, m_tmp_world_quadrangle);

  const katana::World::ObstacleContainer& cont =  m_world->getObstacleContainerPtr();

  #ifdef KATANA_MC_COLLISION_HANDLER
    std::cout << "MC - CollisionHandler: collision: Obstacles in world: " << cont.size() << std::endl;
  #endif
  for( size_t i = 0; i < cont.size();i++)
  {
    if(obstacleInBox(*cont[i], m_tmp_world_quadrangle))
    {
      #ifdef KATANA_MC_COLLISION_HANDLER
        std::cout << "MC - CollisionHandler: collision: Obstacle x= " << cont[i]->getPose().translation()[0] << " , y= " << cont[i]->getPose().translation()[1] << std::endl;
      #endif
      return cont[i];
    }
  }
  return nullptr;
}

bool CollisionHandler::obstacleInBox(const Obstacle& obs, const Quadrangle& q) const
{
  #ifdef KATANA_MC_COLLISION_HANDLER
    std::cout << "MC - CollisionHandler: obstacleInBox: Obstacle x= " << obs.getPose().translation()[0] << " , y= " << obs.getPose().translation()[1] << std::endl;
  #endif
  // Check if center is on line
  if(isWithinOrOnLineOfQuadrangle(obs.getPose().translation(), q))
    return true;
  if(overlapping(obs.getQuadrangle(), q))
    return true;

  return false;
}

bool CollisionHandler::overlapping(const Quadrangle& q1, const Quadrangle& q2) const
{
  // check if one point of q1 is within q2 and vice versa
  for (std::size_t i = 0; i < 4; i++)
  {
    if (isWithinOrOnLineOfQuadrangle(q1[i], q2))
      return true;

    if (isWithinOrOnLineOfQuadrangle(q2[i], q1))
      return true;
  }
  return false;
}

bool CollisionHandler::isWithinOrOnLineOfQuadrangle(const Position2d& point, const Quadrangle& q) const
{

  for (size_t i = 0; i < 4; i++)
  {
    Eigen::Matrix<double, 2, 2> m;
    m.col(0) <<q[(i+1) % 4] - q[i];
    m.col(1) <<point - q[i];

    // If the determinant is negative, the point lies on the left hand to the line
    // and is therefore on the outside of the trapezium (assumed right-handed coordinate system)
    if (m.determinant() < 0.0)
      return false;
  }
  return true;
}

Obstacle::Ptr CollisionHandler::collisionTrajectory(const oadrive::core::Trajectory2d& trajectory,
                                                    std::size_t begin_index, const double& diff_distance,
                                                    std::size_t number_boxes, const Quadrangle& box) const
{
  assert(!trajectory.empty());
  assert(begin_index < trajectory.size());

  Obstacle::Ptr obstacle;

  // first box
  obstacle = collision(box, trajectory[begin_index].getPose());

  if (obstacle != nullptr)
  {
#ifdef KATANA_MC_COLLISION_HANDLER
    std::cout <<"[CollisionHandler] Collision in box no. 0, index: " <<begin_index <<std::endl;
#endif
    return obstacle;
  }

  std::size_t current_index = begin_index + 1;
  std::size_t last_box_index = begin_index;

  if (last_box_index >= trajectory.size())
    return nullptr;

  for (std::size_t i = 1; i < number_boxes; i++)
  {
    while (current_index < trajectory.size())
    {
      if (trajectory.lengthBetween(last_box_index, current_index) > diff_distance)
      {
        break;
      }
      ++current_index;
    }
    if (current_index == trajectory.size())
      return nullptr;

    last_box_index = current_index;
    ++current_index;

#ifdef KATANA_MC_COLLISION_HANDLER
    std::cout <<"[CollisionHandler] Checking box at index " <<last_box_index <<std::endl;
#endif

    // check box
    obstacle = collision(box, trajectory[last_box_index].getPose());

    if (obstacle != nullptr)
    {
#ifdef KATANA_MC_COLLISION_HANDLER
      std::cout <<"[CollisionHandler] Collision in box no. " <<i <<" ,index: " <<last_box_index <<std::endl;
#endif
      return obstacle;
    }

  }

  return nullptr;
}

void CollisionHandler::createWorldCoordinateBox(const Pose2d &p, const Quadrangle &box, Quadrangle &box_world) const
{
  for(u_int8_t i = 0; i < 4; i++)
  {
    box_world[i] = p * box[i];
  }
}

//void CollisionHandler::updateCollisionCalculation()
//{
//  const katana::World::ObstacleContainer& cont =  m_world->getObstacleContainerPtr();
//  for( size_t i; i < cont.size();i++)
//  {
//    if(obstacleInFrontOfCar(cont[i].getQuadrangle()))
//    {
//      m_vehicle_must_stop = true;
//      relevantObstacle = cont[i];
////      if(canChangeDrivingStrip(cont))
////      {
////        m_change_driving_strip = true;
////      }
//      break;
//    }
//  }
//}

//bool CollisionHandler::obstacleInFrontOfCar(Quadrangle obstacle)
//{
//  for(size_t i = 0; i < 4; i++)
//  {
//    boxFrontOfCar[i] = (p * boxFrontOfCar[i]).eval();
//  }

//  if(overlapping(obstacle, boxFrontOfCar))
//  {
//    return true;
//  }
//  return false;
//}

//bool CollisionHandler::canChangeDrivingStrip(katana::World::ObstacleContainer& cont)
//{
//  //obstacle on other driving Strip?
//  assert(false && "changing driving strip in collision handler not implemented");
////  Quadrangle boxOtherDrivingStrip;
////  boxOtherDrivingStrip[0] = Position2d(4500, 1500);
////  boxOtherDrivingStrip[1] = Position2d(4500, -1500);
////  boxOtherDrivingStrip[2] = Position2d(8000, -1000);
////  boxOtherDrivingStrip[3] = Position2d(8000, 1000);

////  for( size_t i; i< cont->size();i++)
////  {
////    Obstacle& curr = cont[i];
////    if(overlapping(curr.getQuadrangle(), boxOtherDrivingStrip))
////    {
////      return true;
////    }
////  }
//  return false;
//}

}
