#include <iostream>

#include "mission_control/maneuver.h"
#include "mission_control/mission_control.h"
#include "RoadBase.h"
#include "Pose.h"
#include "Obstacle.h"

/*************** JUST FOR DEBUGGING *********************/

using namespace std;
using namespace katana;

int main(int argc, char* argv[])
{



  World world;

  world.loadPatches("/home/christoph/Workspace/aadc/src/adtfUser/common/patches/patches.xml");

  Pose2d patch_pose;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(patch_pose, 1.0, 0.0, 0.0);

  Pose2d patch_pose2;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(patch_pose2, 2.0, 0.0, 0.0);

  Pose2d obs;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(obs, 2.0, -0.23, 0.0);

  Pose2d obs2;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(obs2, 2.0, 1.0, 0.0);

  Pose2d obs3;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(obs3, 2.0, 2.0, 2.0);


  world.getMapWriter().addObstacle(obs2 * World::OBSTACLE_IMAGE_POSE.getPose());
  world.getMapWriter().addObstacle(obs3 * World::OBSTACLE_IMAGE_POSE.getPose());

  RoadBase::Ptr patch = world.instantiateRoadBase(PatchType::STRAIGHT, patch_pose, 0);

  world.addPatchToMap(*patch);

  patch = world.instantiateRoadBase(PatchType::STRAIGHT, patch_pose2, 1);

  world.getMapWriter().addObstacle(obs * World::OBSTACLE_IMAGE_POSE.getPose());

  world.addPatchToMap(*patch);

  world.getMapWriter().write("/tmp/test.svg");
  
  //world->addRoadBase(katana::PatchType::STRAIGHT, Pose(0, 0, 0),0);
  //world->addRoadBase(katana::PatchType::STRAIGHT, Pose(10, 10, 0),1);
  //world->addRoadBase(katana::PatchType::STRAIGHT, Pose(20, 20, 0),2);
  //world->addRoadBase(katana::PatchType::STRAIGHT, Pose(30, 30, 0),3);
  //world->addRoadBase(katana::PatchType::STRAIGHT, Pose(40, 40, 0),4);
  //world->addRoadBase(katana::PatchType::STRAIGHT, Pose(45, 45, 0),5);
  //vector<RoadBase::ConstPtr> nextPatches = world->getNextPatches(Pose(0,0,0));
}
