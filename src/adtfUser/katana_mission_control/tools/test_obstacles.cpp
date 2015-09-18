#include <iostream>

#include "mission_control/maneuver.h"
#include "mission_control/mission_control.h"
#include "RoadBase.h"
#include "Pose.h"
#include "Obstacle.h"

/*************** JUST FOR DEBUGGING *********************/

using namespace std;
using namespace katana;


std::list<bool> results;
MissionControl mc;
World::Ptr world;


//! @todo update to pose 2d



/*
// ########## Test cases #########
bool addPatch1() {
  world->addRoadBase(katana::PatchType::STRAIGHT, Pose(0, 0, 0),0);
  vector<RoadBase::ConstPtr> nextPatches = world->getNextPatches(Pose(0,0,0));
  return nextPatches.size() == 1;
}

bool addObstacle1() {
  Pose p = Pose(2,2,0);
  Obstacle::BoundingBox bb;
  bb.first = 1;
  bb.second = 1;
  
  world->addObstacle(std::make_shared<katana::Obstacle>(p,bb,10, ObstacleSource::IR));
  
  return true;
}

bool obstacleIntersectingWithPatch1() {
  RoadBase::ConstPtr patch = world->getNextPatches(Pose(0,0,0)).front();
  bool free0 = world->getObstacleOnDrivingStrip(patch, 0) == nullptr;
  bool free1 = world->getObstacleOnDrivingStrip(patch, 1) == nullptr;
  if(free0) { cout << "Driving strip 0 is free" << endl; } else { cout << "Driving strip 0 is not free" << endl; }
  if(free1) { cout << "Driving strip 1 is free" << endl; } else { cout << "Driving strip 1 is not free" << endl; }
  return free0 && !free1;
}

bool addPatch2() {
  world->addRoadBase(katana::PatchType::STRAIGHT, Pose(0, 0, 0),0);
  vector<RoadBase::ConstPtr> nextPatches = world->getNextPatches(Pose(0,0,0));
  return nextPatches.size() == 1;
}

bool addObstacle2() {
  Pose p = Pose(20,-20,0);
  Obstacle::BoundingBox bb;
  bb.first = 10;
  bb.second = 10;
  
  world->addObstacle(std::make_shared<katana::Obstacle>(p,bb,TIME_TO_LIVE_OBSTACLES, ObstacleSource::IR));
  
  return true;
}

bool obstacleIntersectingWithPatch2() {
  RoadBase::ConstPtr patch = world->getNextPatches(Pose(0,0,0)).front();
  bool free0 = world->getObstacleOnDrivingStrip(patch, 0) == nullptr;
  bool free1 = world->getObstacleOnDrivingStrip(patch, 1) == nullptr;
  if(free0) { cout << "Driving strip 0 is free" << endl; } else { cout << "Driving strip 0 is not free" << endl; }
  if(free1) { cout << "Driving strip 1 is free" << endl; } else { cout << "Driving strip 1 is not free" << endl; }
  return !free0 && free1;
}

*/
// ######## End Testcases ########

void dummy(u_int8_t steering)
{

}

void dummy2(u_int8_t speed)
{

}

void dummy3(const std::vector<katana::RoadBase::ConstPtr>& p1) {
  
}

void dummy4(PerceptionState p1, int32_t p2)
{
  
}

void initialize() {
  /*mc.initialize("/home/philipp/Desktop/aadc/src/adtfUser/katana_mission_control/etc/aadc_route.xml",
                "/home/philipp/Desktop/aadc/src/adtfUser/common/patches/patches.xml",
                std::function<void(u_int8_t)>(dummy),
                std::function<void(u_int8_t)>(dummy2),
                katana::TransmitPatchesFunc(),
		std::function<void(PerceptionState, int32_t)>(dummy4)
                );*/
  world = mc.getSystem()->getWorld();
}

void clearWorld() {
  world->emptyPatches(); 
}

bool allTestsGreen() {
  int counter = 1;
  bool returnValue = true;
  for(bool result : results) {
   if(!result) { 
     cout << "Assertion No.: " << counter << " failed." << std::endl;
     returnValue = false;
    }
    ++counter;
  }
  return returnValue;
}

int main(int argc, char* argv[])
{
  initialize();
  
  // Testcases
  /*
  // Module 1: Creating straight patch on (0,0) and obstacle on (1,1) with bb = 1,1
  // checking if obstacles on driving strip are detected.
  addObstacle1();
  //results.push_back(numberOfObstaclesEquals(1));
  results.push_back(addPatch1());
  results.push_back(obstacleIntersectingWithPatch1());
  clearWorld();
  
  // Module 2: Creating straight patch on (0,0) and obstacle on (20,-20) with bb = 10,10
  // checking if obstacles on driving strip are detected.
  addObstacle2();
  //results.push_back(numberOfObstaclesEquals(1));
  results.push_back(addPatch2());
  results.push_back(obstacleIntersectingWithPatch2());
  clearWorld();
  
  // Module 3: Check deletion of old obstacles
  addObstacle1(); // TTL = 10
  //results.push_back(numberOfObstaclesEquals(1));
  world->setStreamTime(TIME_TO_LIVE_OBSTACLES);
  addObstacle2(); // TTL = TIME_TO_LIVE_OBSTACLES
  //results.push_back(numberOfObstaclesEquals(2));
  world->setStreamTime(TIME_TO_LIVE_OBSTACLES + 11);
  addObstacle1(); // should be deleted
  //results.push_back(numberOfObstaclesEquals(1));
  clearWorld();
  */
  // End Testcases
  if(allTestsGreen()) {
    std::cout << "All tests passed.";
    return 0;
  }
  else {
    std::cout << "Testcase failed.";
    return -1;
  }
}
