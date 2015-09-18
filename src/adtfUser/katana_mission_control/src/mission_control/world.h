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

#ifndef _MISSION_CONTROL_WORLD_H
#define _MISSION_CONTROL_WORLD_H

#include "katanaCommon/katanaCommon.h"
#include "RoadBase.h"
#include "RoadJunction.h"
#include "Obstacle.h"
#include "Pose.h"
#include "ExtendedPoint.h"
#include "Obstacle.h"
#include "RoadSign.h"
#include "mission_control/PositionController.h"
#include "mission_control/ParkingAssistant.h"
#include "mission_control/LanetrackerJobManager.h"
#include "mission_control/maneuver.h"

#ifdef MC_ENABLE_SVG_MAP
#include <oadrive_vision/LocalMapWriter.h>
#endif

#include <list>
#include <array>

using namespace std;

namespace katana
{

class World
{
public:
  //! Pointer shorthand
  typedef std::shared_ptr<World> Ptr;

  //! Const pointer shorthand
  typedef std::shared_ptr<const World> ConstPtr;

  //! Container holding all patches
  typedef std::vector<RoadBase::Ptr> RoadPatchContainer;
  typedef std::vector<RoadBase::ConstPtr> RoadPatchContainerConst;

  //! Container holding all obstacles
  typedef std::vector<Obstacle::Ptr> ObstacleContainer;
  typedef std::shared_ptr<ObstacleContainer> ObstacleContainerPtr;


  //! Contructor
  World();

  //! Destructor
  virtual ~World();

  //! Set PositionController
  void initialize(const PositionController::ConstPtr& position_controller, const Maneuver::ConstPtr maneuver)
  {
    m_position_controller = position_controller;
    m_maneuver = maneuver;
  }

  //! Read flag if patches have been updated
  bool havePatchesBeenUpdated() const           { return m_patches_updated; }
  //! Set flag to false when patches have been read
  void trajectoryGeneratedFromPatches()                    { m_patches_updated = false; }

  _clock_time getStreamTime()		     { return m_current_streamTime; }
  void setStreamTime(_clock_time current_streamTime)	{ m_current_streamTime = current_streamTime; }

  //! load Patches from specified XML-Files (you need to call this function ONCE), returns false on error in any file
  bool loadPatches(const std::string& m_filename);

  //! ID and Pose from message -> Create new RoadSegment from Patch template and add to world, birth_stamp used to check age of patch (driven distance)
  //! This only instantiates an appropriate roadbase::ptr and calls an overload of this function
  //! Returns true if new patch; returns false if patch is already known
  bool addRoadBase(PatchType type, const Pose2d& pose, u_int32_t id, double detection_stamp = std::numeric_limits<double>::infinity(), double matching_value = 0.0);

  //! Add an already instantiated RoadBase
  //! Returns true if new patch; returns false if patch is already known
  bool addRoadBase(RoadBase::Ptr roadbase, double detection_stamp = std::numeric_limits<double>::infinity());

  //! New patches from lanetracker
  void addNewPatches(const PatchVectorPtr& patch_vector, PerceptionState perception_state);

  u_int32_t getLastPatchId();

  //! Delete patches when the car has driven more than DISTANCE_TO_DELETE_PATCHES since their last detection
  void removeOldPatches(double current_driven_distance);

  void addObstacle(Obstacle::Ptr obstacle);

  void removeOldObstacles();

  //! Save right of way of junctions due to traffic signs
  //void saveRightofWay(TrafficSign sign);

  //! Set the last saved
  //void setJunctionRightOfWay();

  //! delete all known patches
  void emptyPatches()			{ m_roadbases.clear(); }

  //! Delete all patches, but not the junction @todo delete only patches in front of car
  void deleteAllPatchesButJunction();

  //! delet all knwon obstacles
  void emptyObstacles()			{ m_obstacles.clear(); }

  // get Next Patches to current pose
  RoadPatchContainerConst getNextPatches() const;

  //! get patch next ot pose
  RoadBase::ConstPtr getPatchNextToPosition(const oadrive::core::Position2d& position) const;

//  //! Get the obstacle that is on a given driving strip. Returns nullptr if no obstacle
//  Obstacle::Ptr getObstacleOnDrivingStrip(RoadBase::ConstPtr roadBase, RoadBase::DrivingStripId drivingStrip);

  //! Get constant patch templates
  const RoadPatch::PatchArray& getPatchTemplates() const     { return *m_patches; }

  //! Instantiate roadBase
  RoadBase::Ptr instantiateRoadBase(PatchType type, const Pose2d &pose, u_int32_t id, bool isVirtual = false);

  //! Instantiate RoadJunction
  RoadJunction::Ptr instantiateRoadJunction(PatchType type, const Pose2d &pose, u_int32_t id);

  //! Set the direction for the last inserted junction
  void setDirection(Action nextDirection);

  const ObstacleContainer& getObstacleContainerPtr() const { return m_obstacles; }

  #ifdef MC_ENABLE_SVG_MAP
  //! Return MapWriter object for other modules to draw stuff
  oadrive::vision::LocalMapWriter& getMapWriter()     { return m_map_writer; }

  //! Add RoadPatch to LocalMapWriter
  void addPatchToMap(const RoadBase& patch);
  #endif

  //! Search for parking spot, returns Object to ask for success
  ParkingAssistant::Ptr searchForParkingSpot(ParkingSpace parking_space_type, SideOfVehicle side = RIGHT, u_int32_t search_for_empty = 0);
  void disableSearchForParkingSpot();

  static const oadrive::core::ExtendedPose2d OBSTACLE_IMAGE_POSE;
private:

  //! PositionController instance, get information about vehicle pose and driven distance
  PositionController::ConstPtr m_position_controller;
  //! Maneuver
  Maneuver::ConstPtr m_maneuver;

  //! Get the last inserted junction
  RoadBase::Ptr getLastJunction();

  //! Check if Obstacle is older than TIME_TO_LIVE_OBSTACLES
  bool isObstacleOutdated(Obstacle::ConstPtr obstacle);

  #ifdef KATANA_WORLD_DEBUG_PLOT
  void writePatchesToGnuPlot() const;
  void writeObstacleToFile(std::ofstream* gnuplot_file, Obstacle::Ptr obstacle);
  #endif

  //!
  //! \brief evaluatePose
  //! \param p: pose to evaluate
  //! \return score, 0 if bad, higher is better
  //!
  u_int32_t evaluatePose(const Pose2d& p, float factor = 1) const;

  //! All patches
  RoadPatchContainer m_roadbases;

  //! All Obstacles
  ObstacleContainer m_obstacles;

  //! Known patches
  std::shared_ptr<RoadPatch::PatchArray> m_patches;

  //! Flag if new patches have arrived
  bool m_patches_updated;

  _clock_time m_current_streamTime;


  //! Last seen traffic sign
  TrafficSign m_last_traffic_sign;

  //! ParkingAssistant when searching for parking spot
  ParkingAssistant::Ptr m_parking_assistant;

  #ifdef MC_ENABLE_SVG_MAP
  //! LocalMapWriter
  oadrive::vision::LocalMapWriter m_map_writer;
  #endif

  //! Filenames
  static const char* PATCH_FILENAME[];
  //! Poses for image placement
  static const std::array<oadrive::core::ExtendedPose2d, (u_int32_t)PatchType::PATCH_COUNT> PATCHFILE_POSES;
  static const std::array<oadrive::core::Position2d, (u_int32_t)PatchType::PATCH_COUNT> PATCHFILE_SIZES;
};


} //ns

#endif // _MISSION_CONTROL_WORLD_H
