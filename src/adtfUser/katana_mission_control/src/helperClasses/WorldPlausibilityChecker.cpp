#include "WorldPlausibilityChecker.h"
#include <math.h>
#include <boost/concept_check.hpp>

namespace katana
{
WorldPlausibilityChecker::WorldPlausibilityChecker()
{

}

WorldPlausibilityChecker::~WorldPlausibilityChecker()
{

}

void WorldPlausibilityChecker::check_ids(World::RoadPatchContainer const roadbases)
{
  // check if ids are correctly set
  // id's should be unique and ordered in vector
  int32_t last_id = -1;
  for(RoadBase::ConstPtr roadBase : roadbases) {
    if((int32_t)roadBase->getId() <= last_id) {
      assert(false && "Error: wrong ids in world. See WorldPlausibilityChecker.cpp");
    }
    last_id = roadBase->getId();
  }
}

void WorldPlausibilityChecker::check_ids(const vector< RoadBase::ConstPtr > roadbases)
{
  // check if ids are correctly set
  // id's should be unique and ordered in vector
  int32_t last_id = -1;
  for(RoadBase::ConstPtr roadBase : roadbases) {
    if((int32_t)roadBase->getId() <= last_id) {
      assert(false && "Error: wrong ids in world. See WorldPlausibilityChecker.cpp");
    }
    last_id = roadBase->getId();
  }
}

bool WorldPlausibilityChecker::plausibility_check_for_new_Patch(RoadBase::ConstPtr const newRoadbase, World::RoadPatchContainer const roadbases)
{
  // If no roadbase is already known, then the new one is ok.
  if(roadbases.size() == 0) {
    return true;    
  }
  
  /*Pose2d endpose = roadbases.back()->getEndPose();
  // Check if new patch is not to far away from last patch
  if(endpose.distanceToPoint(newRoadbase->getAnchorPose()) > MAX_DISTANCE_BETWEEN_PATCHES) {
    assert(false && "World: Patch is to far away from last patch. See WorldPlausibilityChecker.cpp");
    return false;
  }
  
  // Check if new angle is not
  if(std::abs(endpose.getTheta() - newRoadbase->getAnchorPose().getTheta()) > MAX_ANGLE_BETWEEND_END_AND_START_POSE) {
    assert(false && "World: angle of start pose differs to much from angle of end pose. See WorldPlausibilityChecker.cpp");
    return false;
  }*/
  
  #ifndef NDEBUG
  check_ids(roadbases);
  #endif
  
  return true;
}

bool WorldPlausibilityChecker::plausibility_check_for_known_Patch(RoadBase::ConstPtr const newRoadbase, RoadBase::ConstPtr const oldRoadbase, World::RoadPatchContainer const roadbases)
{  
  // Check if patchtypes are equal
  /*if(newRoadbase->getPatchType() != oldRoadbase->getPatchType()) {
    assert(false && "Patch type of old patch does not match new type. See WorldPlausibilityChecker.cpp");
    return false;
  }*/
  
  // Check if poses are similar
  _position_type xdiff = std::abs(newRoadbase->getAnchorPose().translation().x() - oldRoadbase->getAnchorPose().translation().x());
  _position_type ydiff = std::abs(newRoadbase->getAnchorPose().translation().y() - oldRoadbase->getAnchorPose().translation().y());
  //_angle_type angleDiff = std::abs(newRoadbase->getAnchorPose().getTheta() - oldRoadbase->getAnchorPose().getTheta());
  if(xdiff > MAX_X_DIFF_SAME_PATCHES) {
    assert(false && "Difference in x coordinate between old and new patch is to big. See WorldPlausibilityChecker.cpp");
    return false;
  }
  if(ydiff > MAX_Y_DIFF_SAME_PATCHES) {
    assert(false && "Difference in y coordinate between old and new patch is to big. See WorldPlausibilityChecker.cpp");
    return false;
  }
  /*if(angleDiff > MAX_ANGLE_DIFF_SAME_PATCHES) {
    assert(false && "Difference in theta between old and new patch is to big. See WorldPlausibilityChecker.cpp");
    return false;
  }*/
  
  #ifndef NDEBUG
  check_ids(roadbases);
  #endif
  
  return true;
}

  }