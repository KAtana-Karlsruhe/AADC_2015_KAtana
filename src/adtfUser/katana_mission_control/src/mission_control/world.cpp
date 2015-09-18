// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \author  Philipp Hertweck <hertweck@fzi.de>
 * \date    2014-12-04
 *
 */
//----------------------------------------------------------------------

#include "world.h"
#include "System.h"
#ifdef KATANA_MISSION_CONTROL_ENABLE_PATCH_PLAUSIBILITY_CHECKS
  #include <helperClasses/WorldPlausibilityChecker.h>
#endif
#include <stdint.h>
#include <limits.h>

#ifdef KATANA_WORLD_DEBUG_PLOT
#include <fstream>
#endif

namespace katana
{

using namespace oadrive::core;

const char* World::PATCH_FILENAME[] =
{
  "/tmp/straight.png",            //STRAIGHT = 0
  "/tmp/small_l_curve.png",       //SMALL_L_CURVE = 1
  "/tmp/small_r_curve.png",        //SMALL_R_CURVE = 2
  "/tmp/kreuzung_start_junction_450_600.png"  //JUNCTION
};

const std::array<ExtendedPose2d, (u_int32_t)PatchType::PATCH_COUNT> World::PATCHFILE_POSES =
    std::array<ExtendedPose2d, (u_int32_t)PatchType::PATCH_COUNT>
{
  ExtendedPose2d(0.61, 0.53, -M_PI/2),
  ExtendedPose2d(0.61, 0.53, -M_PI/2),
  ExtendedPose2d(0.61, 0.53, -M_PI/2),
  ExtendedPose2d(2.0, 1.5, -M_PI/2)
};

const std::array<oadrive::core::Position2d, (u_int32_t)PatchType::PATCH_COUNT> World::PATCHFILE_SIZES =
    std::array<oadrive::core::Position2d, (u_int32_t)PatchType::PATCH_COUNT>
{
  Position2d(0.61, 1.06),
  Position2d(0.61, 1.06),
  Position2d(0.61, 1.06),
  Position2d(3.0, 3.0)
};

const oadrive::core::ExtendedPose2d World::OBSTACLE_IMAGE_POSE = ExtendedPose2d(0.1 ,0.15, 0.0);

World::World()
  : m_patches_updated(false)
  , m_current_streamTime(0)
  , m_last_traffic_sign(TrafficSign::UNKNOWN)
{
}

World::~World()
{
}

// ########### Management of Patches ##########################

bool World::loadPatches(const std::string& m_filename)
{
  m_patches = RoadPatch::loadPatches(m_filename);
  return m_patches != nullptr;
}

RoadBase::Ptr World::instantiateRoadBase(PatchType type, const Pose2d &pose, u_int32_t id, bool isVirtual) {
  return std::make_shared<RoadBase>((*m_patches)[(u_int32_t)type], pose, id, isVirtual);
}

RoadJunction::Ptr World::instantiateRoadJunction(PatchType type, const Pose2d& pose, u_int32_t id)
{
  return std::make_shared<RoadJunction>((*m_patches)[(u_int32_t)type], pose, id);
}

bool World::addRoadBase(PatchType type, const Pose2d &pose, u_int32_t id, double detection_stamp, double matching_value)
{
  if (type >= PatchType::PATCH_COUNT)   //< error
    return false;

  RoadBase::Ptr segment;
  if(type == PatchType::JUNCTION) {
    segment = instantiateRoadJunction(type, pose, id);
  } else {
    // create RoadSegment from template and pose
   segment = instantiateRoadBase(type, pose, id);
  }

  segment->matchingValue() = matching_value;

  // RoadBase created, add to world
  return addRoadBase(segment, detection_stamp);
}

bool World::addRoadBase(RoadBase::Ptr roadbase, double detection_stamp)
{
  // set stamp
  roadbase->setDetectionStamp(detection_stamp);
  // now we have an updated patch
  m_patches_updated = true;


  // Check if patch with id is already known
  bool alreadyInserted = false;

  for(RoadPatchContainer::iterator it = m_roadbases.begin(); it!=m_roadbases.end(); ++it)
  {
    // Patch with id found -> override old patch
    if((*it)->getId() == roadbase->getId()) {
      #ifdef KATANA_MISSION_CONTROL_ENABLE_PATCH_PLAUSIBILITY_CHECKS
  WorldPlausibilityChecker::plausibility_check_for_known_Patch(segment, (*it), m_roadbases);
      #endif

      // bool save virtual status
      if((*it)->isVirtual()) {
      	roadbase->setVirtual();
      }

      // save direction for junction
      Action savedDirection = Action::UNKNOWN;
      if(roadbase->getPatchType() == PatchType::JUNCTION) {
	savedDirection = (*it)->getDirection();
      }
      (*it) = roadbase;
      if(roadbase->getPatchType() == PatchType::JUNCTION) {
	(*it)->setDirection(savedDirection);
      }

      alreadyInserted = true;

			#ifdef KATANA_MC_WORLD_DEBUG
	std::cout << "Updated old patch in world with new one: ";
	segment->printToConsole();
			#endif

      break;
    }
  }

  // Patch with given id not found yet -> insert
  if(!alreadyInserted) {
    #ifdef KATANA_MISSION_CONTROL_ENABLE_PATCH_PLAUSIBILITY_CHECKS
      // Check if received patch is plausible to already received patches
      WorldPlausibilityChecker::plausibility_check_for_new_Patch(segment, m_roadbases);
    #endif

    #ifdef KATANA_MC_WORLD_DEBUG
      std::cout << "Added new Patch to world ";
      segment->printToConsole();
    #endif

    // Insert new Roadbase
    m_roadbases.push_back(roadbase);
  }

  if(alreadyInserted) {
    return false;
  } else {
    return true;
  }
}

void World::addNewPatches(const PatchVectorPtr& patch_vector, PerceptionState perception_state)
{
  bool newPatch;

  for (size_t i = 0; i < patch_vector->size(); i++)
  {
    PatchType type = (PatchType)(*patch_vector)[i].patch_type;
    // add to world
    newPatch = addRoadBase(type , (*patch_vector)[i].sp.toPose2dScaled(),
                                                 (*patch_vector)[i].id,
                                                 m_position_controller->getOverallDrivenDistance(),
                                                 (*patch_vector)[i].match_value);

    // if type is junction and junction not known yet
    if(type == PatchType::JUNCTION && newPatch) {

      // set direction
      Action nextDirection = m_maneuver->getCurrentManeuver();
      if(nextDirection == Action::LEFT || nextDirection == Action::RIGHT || nextDirection == Action::STRAIGHT) {
        setDirection(nextDirection);
      }

      // set the saved traffic sign to the junction
      //setJunctionRightOfWay();
    }
  }
}

u_int32_t World::getLastPatchId()
{
  if(m_roadbases.empty()) {
    return 0;
  } else {
    return m_roadbases.back()->getId();
  }
}

void World::setDirection(Action nextDirection)
{
  getLastJunction()->setDirection(nextDirection);
}

RoadBase::Ptr World::getLastJunction() {
  for(RoadPatchContainer::reverse_iterator it = m_roadbases.rbegin(); it != m_roadbases.rend(); ++it) {
    if((*it)->isJunction()) {
      return (*it);
    }
  }

  assert(false && "called getLastJunction, but no junction found yet");
  return nullptr;
}

World::RoadPatchContainerConst World::getNextPatches() const
{
  #ifdef KATANA_MC_WORLD_DEBUG
    std::cout << "World returning patches" << endl;
    for(RoadPatchContainer::const_iterator it = m_roadbases.begin(); it != m_roadbases.end(); ++it)
    {
      std::cout <<(*it)->getAnchorPose() <<std::endl;
    }
  #endif

  // Gnuplot
  #ifdef KATANA_WORLD_DEBUG_PLOT
  writePatchesToGnuPlot();
  #endif

  World::RoadPatchContainerConst patches(m_roadbases.begin(),m_roadbases.end());

  return patches;
}

RoadBase::ConstPtr World::getPatchNextToPosition(const oadrive::core::Position2d& position) const
{
  if (m_roadbases.empty())
    return nullptr;

  double min_dist = std::numeric_limits<double>::max();
  std::size_t best = 0;

  for (std::size_t i = 0; i < m_roadbases.size(); i++)
  {
    const double dist = (m_roadbases[i]->getAnchorPose().translation() - position).norm();
    if (dist < min_dist)
    {
      best = i;
      min_dist= dist;
    }
  }

  return m_roadbases[best];
}

void World::removeOldPatches(double current_driven_distance)
{
  for(RoadPatchContainer::iterator it = m_roadbases.begin(); it != m_roadbases.end() ; /* erase in loop */)
  {
    if (current_driven_distance - (*it)->getDetectionStamp() > DISTANCE_TO_DELETE_PATCHES)
    {
#ifdef KATANA_MC_WORLD_DEBUG
    std::cout << "[MissionControl::World::removeOldPatches] - Removed old Patch from World ";
    (*it)->printToConsole();
#endif
#ifdef MC_ENABLE_SVG_MAP
      addPatchToMap(**it);
#endif

      // remove from vector
      it = m_roadbases.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

#ifdef MC_ENABLE_SVG_MAP
void World::addPatchToMap(const RoadBase& patch)
{
  // add image to local map, because it won't be updated anymore
  const u_int32_t p_type = (u_int32_t)patch.getPatchType();
  oadrive::core::Pose2d p = patch.getAnchorPose() * PATCHFILE_POSES[p_type].getPose();
  m_map_writer.addImage(p, PATCH_FILENAME[p_type], PATCHFILE_SIZES[p_type].y(), PATCHFILE_SIZES[p_type].x());
}
#endif

u_int32_t World::evaluatePose(const Pose2d &p, float factor) const
{
  // kind elliptic distance function... TODO: improve this function!
  double distance = Position2d(p.translation().x(), p.translation().y() *2).norm();

  if (distance > 0.3 * factor)
    return 0;

  return 0.4 * factor - distance;
}

#ifdef KATANA_WORLD_DEBUG_PLOT
void World::writePatchesToGnuPlot() const {
  std::ofstream gnuplot_file("/tmp/world_gnuplot.gpldata",std::ios::out);
  gnuplot_file <<"#Plot this file with: plot <filename> using 1:2 with lp" <<std::endl
		<<"#set size ratio -1" <<std::endl;

  for(RoadBase::Ptr patch : m_roadbases) {
    gnuplot_file << patch->getAnchorPose().translation().x() << " " << patch->getAnchorPose().translation().y();
    gnuplot_file << " " << PoseTraits<Pose2d>::yaw(patch->getAnchorPose());
    gnuplot_file << " " << static_cast<std::underlying_type<katana::PatchType>::type>(patch->getPatchType());
    gnuplot_file << " " << patch->getId() << std::endl;
  }

  gnuplot_file.close();
}
#endif

// ########### Management of Obstacles ##########################

void World::addObstacle(Obstacle::Ptr obstacle)
{
  m_obstacles.push_back(obstacle);

  if (m_parking_assistant != nullptr)
  {
    if (obstacle->getSource() == IR_FRONT_RIGHT_LONG || obstacle->getSource() == IR_FRONT_RIGHT_SHORT || obstacle->getSource() == IR_REAR_RIGHT_SHORT)
    {
      m_parking_assistant->addSignificantObstacle(obstacle,
                                              m_position_controller->getCurrentVehiclePose(),
                                              m_position_controller->getOverallDrivenDistance());
    }
  }

#ifdef KATANA_MC_WORLD_OBSTACLE_DEBUG
  std::cout << "MC - World: add obstacle: Obstacle source " << obstacle->getSource() << " ,x= " << obstacle->getPose().translation()[0] << " , y= " << obstacle->getPose().translation()[1] << std::endl;
#endif
  // Gnuplot
#ifdef KATANA_WORLD_DEBUG_PLOT
  std::ofstream gnuplot_file("/tmp/world_obstacles_gnuplot.gpldata",std::ios::out);
  //gnuplot_file.open("/tmp/world_gnuplot.gpldata");
  gnuplot_file <<"#Plot this file with: plot <filename> using 1:2 with lp" << std::endl
               <<"#set size ratio -1" << std::endl
               << "#x y theta xDimension yDimension" << std::endl;
  writeObstacleToFile(&gnuplot_file,obstacle);
  gnuplot_file.close();
  #endif
}

#ifdef KATANA_WORLD_DEBUG_PLOT
void World::writeObstacleToFile(std::ofstream* gnuplot_file, Obstacle::Ptr obstacle)
{
  (*gnuplot_file) << obstacle->getPose().translation().x() << " " << obstacle->getPose().translation().y();
  (*gnuplot_file) << " " << PoseTraits<Pose2d>::yaw(obstacle->getPose());
  (*gnuplot_file) << " " << obstacle->getBoundingBox().first << " " << obstacle->getBoundingBox().second << std::endl;
}
#endif

//Obstacle::Ptr World::getObstacleOnDrivingStrip(RoadBase::ConstPtr roadBase, RoadBase::DrivingStripId drivingStrip)
//{
//  if((!isObstacleOutdated(m_obstacles.first)) && (roadBase->isObstacleIntersectingWithDrivingStrip(m_obstacles.first, drivingStrip))) {
//    return m_obstacles.first;
//  } else if((!isObstacleOutdated(m_obstacles.second)) && (roadBase->isObstacleIntersectingWithDrivingStrip(m_obstacles.second, drivingStrip))) {
//    return m_obstacles.second;
//  }

//  return nullptr;
//}

void World::removeOldObstacles()
{
  for(ObstacleContainer::iterator it = m_obstacles.begin(); it != m_obstacles.end() ; /* erase in loop */)
  {
    if(isObstacleOutdated(*it))
    {
#ifdef KATANA_MC_WORLD_DEBUG
      std::cout << "[MissionControl::World::removeOldObstacles] - Removed old Obstacle from World ";
     (*it)->printToConsole();
#endif
      // remove from vector
#ifdef MC_ENABLE_SVG_MAP
      Pose2d tmp = (*it)->getPose();
      PoseTraits<Pose2d>::fromOrientationRPY(tmp, 0.0);
      m_map_writer.addObstacle(tmp * OBSTACLE_IMAGE_POSE.getPose());
#endif
      it = m_obstacles.erase(it);
    } else {
      ++it;
    }
  }
}

bool World::isObstacleOutdated(Obstacle::ConstPtr obstacle)
{
  if((m_current_streamTime - obstacle->getStamp()) > TIME_TO_LIVE_OBSTACLES)  {
    #ifdef KATANA_MC_WORLD_OBSTACLE_DEBUG
      std::cout << "MC - World: outdated obstacle: Obstacle x= " << obstacle->getPose().translation()[0] << " , y= " << obstacle->getPose().translation()[1] << std::endl;
    #endif
    return true;
  }
  return false;
}

/*
// ############## Traffic Signs ##############/
void World::saveRightofWay(TrafficSign sign)
{
  m_last_traffic_sign = sign;
}

void World::setJunctionRightOfWay()
{
  assert(m_last_traffic_sign != TrafficSign::UNKNOWN && "World: last traffic sign is unkown. Detected junction but traffic sign not known.");
  RoadJunction::Ptr junction = std::static_pointer_cast<RoadJunction>(getLastJunction());
  junction->setRightOfWay(m_last_traffic_sign);
  m_last_traffic_sign = TrafficSign::UNKNOWN;
}
*/

void World::deleteAllPatchesButJunction()
{
  RoadBase::Ptr junction;

  for (RoadPatchContainer::iterator it = m_roadbases.begin(); it != m_roadbases.end(); it++)
  {
    if ((*it)->isJunction())
    {
      junction = *it;

      // if we have two junctions, write the older one
      #ifdef MC_ENABLE_SVG_MAP
	if(junction != nullptr) {
	  addPatchToMap(*junction);
	}
      #endif
    }
    // write all patches except for junction to map
    #ifdef MC_ENABLE_SVG_MAP
      else {
	addPatchToMap(**it);
      }
    #endif
  }

  assert(junction != nullptr && "junction != nullptr");

  m_roadbases.clear();
  m_roadbases.push_back(junction);
}

ParkingAssistant::Ptr World::searchForParkingSpot(ParkingSpace parking_space_type, SideOfVehicle side, u_int32_t search_for_empty)
{
  //assert(!m_parking_assistant && "World: you did not disable the previous search for parking spot");

  m_parking_assistant.reset(new ParkingAssistant());
  m_parking_assistant->enableSearching(parking_space_type, m_position_controller->getOverallDrivenDistance(), side, search_for_empty);

  return m_parking_assistant;
}

void World::disableSearchForParkingSpot()
{
  //assert(m_parking_assistant != nullptr && "World: no search for parking spot currently active");

  m_parking_assistant.reset();
}

} //ns
