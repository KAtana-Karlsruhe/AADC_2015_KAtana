// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-14
 *
 */
//----------------------------------------------------------------------

#ifndef _KATANA_WORLD_ROADPATCH_H
#define _KATANA_WORLD_ROADPATCH_H

#include <vector>

#include "katanaCommon/katanaCommon.h"
#include "tinyxml2.h"

#include <oadrive_core/Pose.h>
#include <oadrive_core/Trajectory2d.h>

using namespace oadrive::core;

namespace katana
{

/**
 * @brief The RoadPatch class
 * Load patch from XML definition file. After reading in constructor,
 * the RoadPatch instances are constant and used to create
 * @see RoadSegment.
 */
class RoadPatch
{
public:
  enum class CORNER_POINT : u_int8_t {
    UP_LEFT,
    UP_RIGHT,
    DOWN_RIGHT,
    DOWN_LEFT,
    CORNER_POINT_COUNT = 4
  };

  enum class STRIP_TYPE : u_int8_t {
    RIGHT = 0,
    LEFT = 1
  };

  /** ************ TYPES **************/
  //! Const shared pointer shorthand
  typedef std::shared_ptr<const RoadPatch> ConstPtr;
  //! Array containing all patches
  typedef std::array<RoadPatch::ConstPtr, (u_int32_t)PatchType::PATCH_COUNT> PatchArray;

  //! Driving strip polygon, relative to anchor pose
  typedef std::shared_ptr<Trajectory2d> TrajectoryPtr;
  typedef std::shared_ptr<const Trajectory2d> TrajectoryConstPtr;
  
  typedef std::vector<TrajectoryPtr> TrajectoryContainer;
  
  typedef std::vector<TrajectoryConstPtr> TrajectoryConstContainer;
  typedef std::shared_ptr<TrajectoryConstContainer> TrajectoryConstContainerPtr;
  typedef std::shared_ptr<const TrajectoryConstContainer> TrajectoryConstContainerConstPtr;

  //! End points
  typedef std::vector<Pose2d> PoseContainer;
  typedef std::shared_ptr<PoseContainer> EndPoseContainerPtr;
  typedef std::shared_ptr<const PoseContainer> EndPoseContainerConstPtr;
  
  //! Corner points
  typedef std::array<Position2d, (u_int32_t)CORNER_POINT::CORNER_POINT_COUNT> PatchBoundary;
  typedef std::shared_ptr<PatchBoundary> PatchBoundaryPtr;
  typedef std::shared_ptr<const PatchBoundary> PatchBoundaryConstPtr;

  //! Driving strips by diversions
  typedef std::vector<TrajectoryContainer> DiversionContainer;
  typedef std::vector<TrajectoryConstContainer> DiversionConstContainer;
  typedef std::shared_ptr<DiversionContainer> DiversionContainerPtr;
  typedef std::shared_ptr<const DiversionContainer> DiversionContainerConstPtr;
  typedef std::shared_ptr<const DiversionConstContainer> DiversionConstContainerConstPtr;
  typedef std::shared_ptr<DiversionConstContainer> DiversionConstContainerPtr;

  //! Load all patches from one file, returns nullptr on error
  static std::shared_ptr<PatchArray> loadPatches(const std::string& filename);

  //! Constructor loads from XML (first occurence of XML_TITLE_ELEMENT)
  RoadPatch(const std::string& filename);

  //! Constructor: load from section in file defined by XML-Title-Element
  RoadPatch(tinyxml2::XMLElement* titleElement);

  //! Disable default constructor
  RoadPatch() = delete;
  //! Disable copying
  RoadPatch& operator=(const RoadPatch& rhs) = delete;

  //! Destructor
  virtual ~RoadPatch()  {}

  //! Good flag
  bool good() const     { return m_good; }

  //! Read access, return default driving strip
  TrajectoryConstPtr getDrivingStrip() const                     { return m_strip_collection->at(0); }
  //! Driving Strips
  TrajectoryConstContainerConstPtr getDrivingStripContainer() const   { return m_strip_collection; }
  //! Boundary points
  PatchBoundaryConstPtr getPatchBoundary() const                   { return m_patch_boundary; }
  //! Endpose Container
  EndPoseContainerConstPtr getEndPoseContainer() const		   { return m_endpose_collection; }
  //! Strip roles
  DiversionConstContainerConstPtr getDiversionContainer() const         { return m_diversion_collection; }

  PatchType getPatchType() const                  { return m_patch_type; }
  u_int32_t getPatchTypeInt() const             { return (u_int32_t)m_patch_type; }

  //! XML Constants
  static const constexpr char* XML_TITLE_ELEMENT = "patch";
  static const constexpr char* XML_TITLE_ID_= "id";

  static const constexpr char* XML_DIVERSION_ELEMENT = "diversion";
  static const constexpr char* XML_DIVERSION_ID = "id";
  static const constexpr char* XML_DIVERSION_DIRECTION_ATTRIB = "reverse";

  static const constexpr char* XML_STRIP_ELEMENT = "strip";
  static const constexpr char* XML_STRIP_ID = "id";

  static const constexpr char* XML_ENDPOSE_ELEMENT = "endpose";
  static const constexpr char* XML_ENDPOSE_X = "xvalue";
  static const constexpr char* XML_ENDPOSE_Y = "yvalue";
  static const constexpr char* XML_ENDPOSE_THETA = "theta";

  static const constexpr char* XML_POINT_ELEMENT = "point";
  static const constexpr char* XML_POINT_X = "xvalue";
  static const constexpr char* XML_POINT_Y = "yvalue";
  static const constexpr char* XML_POINT_YAW = "yaw";

  static const constexpr char* XML_CORNER_ELEMENT = "corner";
  static const constexpr char* XML_CORNER_XVALUE = "xvalue";
  static const constexpr char* XML_CORNER_YVALUE = "yvalue";
  static const constexpr char* XML_CORNER_ID = "id";
  static const char* XML_CORNER_IDS[];

protected:

  bool fillData(tinyxml2::XMLElement* titleElement);

  //! Driving strips
  TrajectoryConstContainerPtr m_strip_collection;
  DiversionConstContainerPtr m_diversion_collection;

  //! Number of diversions of this patch (e.g. normal road section: diversion = 1, e.g. 4-Way-Junction: diversion = 3)
  u_int8_t m_number_of_diversions;

  //! End poses of this patch
  EndPoseContainerPtr m_endpose_collection;

  //! Boudary rectangle
  PatchBoundaryPtr m_patch_boundary;

  //! Type of patch
  PatchType m_patch_type;

  //! Reading successful
  bool m_good;

private:

};


} // ns

#endif //_KATANA_WORLD_ROADPATCH_H
