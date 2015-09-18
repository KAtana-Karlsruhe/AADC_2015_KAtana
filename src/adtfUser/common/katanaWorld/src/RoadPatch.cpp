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
#include <map>

#include "RoadPatch.h"

using namespace tinyxml2;

namespace katana
{

const char* RoadPatch::XML_CORNER_IDS[] =  // same order as CORNER_POINT enum!
{
  "up-left",
  "up-right",
  "down-right",
  "down-left",
};

std::shared_ptr<RoadPatch::PatchArray> RoadPatch::loadPatches(const std::string& filename)
{
  XMLDocument doc;
  doc.LoadFile(filename.c_str());

  if (doc.Error())
    return nullptr;

  std::shared_ptr<PatchArray> arr = std::make_shared<PatchArray>();

  for (XMLElement* titleElement = doc.FirstChildElement(XML_TITLE_ELEMENT);
       titleElement != nullptr;
       titleElement = titleElement->NextSiblingElement(XML_TITLE_ELEMENT))
  {
    // Create new
    ConstPtr tmp = std::make_shared<RoadPatch>(titleElement);

    // Check for XML/Read error
    if (!tmp->good())
      return nullptr;

    // Save according to ID
    (*arr)[tmp->getPatchTypeInt()] = tmp;
  }

  return arr;
}

RoadPatch::RoadPatch(const std::string& filename)
  : m_good(false)
{
  XMLDocument doc;
  doc.LoadFile(filename.c_str());

  if (doc.Error())
    return;

  XMLElement* titleElement = doc.FirstChildElement(XML_TITLE_ELEMENT);

  m_good = fillData(titleElement);
}

RoadPatch::RoadPatch(XMLElement* titleElement)
{
  m_good = fillData(titleElement);
}

bool RoadPatch::fillData(XMLElement* titleElement)
{
  m_number_of_diversions = 0;

  // create strip collection and end pose collection of this roadPatch instance
  m_strip_collection = std::make_shared<TrajectoryConstContainer>();    //< all strips
  m_diversion_collection = std::make_shared<DiversionConstContainer>();   //< driving strips by diversion
  m_endpose_collection = std::make_shared<PoseContainer>();

  // check titleElement
  int tmp = titleElement->IntAttribute(XML_TITLE_ID_);
  if (tmp >= (int)PatchType::PATCH_COUNT)  //< Error non valid ID
    return false;
  m_patch_type = (PatchType)tmp;

  //****** ITERATE OVER DIVERSIONS *******/
  u_int8_t current_diversion_id = 0;
  XMLElement* diversion = titleElement->FirstChildElement(XML_DIVERSION_ELEMENT);

  while( diversion != nullptr )
  {
    // read current diversion id
    int32_t diversion_id = diversion->IntAttribute(XML_DIVERSION_ID);

    if (diversion_id < 0 || diversion_id > 255)
      return false;   //< error invalid diversion ID

    // read tags in correct order, skip wrong ids first
    if ((u_int8_t)diversion_id != current_diversion_id)
    {
      diversion = diversion->NextSiblingElement(XML_DIVERSION_ELEMENT);
      continue;
    }

    // Read diversion direction attribute
    int32_t reverse_tag;
    bool forward = true;
    if (diversion->QueryIntAttribute(XML_DIVERSION_DIRECTION_ATTRIB, &reverse_tag) == tinyxml2::XML_SUCCESS)
    {
      if (reverse_tag == 1)
        forward = false;
    }

    TrajectoryConstContainer current_diversion;


    //****** ITERATE OVER STRIPS ***********/
    u_int8_t current_strip_id = 0;
    XMLElement* strip = diversion->FirstChildElement(XML_STRIP_ELEMENT);
    while ( strip != nullptr )
    {
      // read current strip id
      int32_t strip_id = strip->IntAttribute(XML_STRIP_ID);
      if (strip_id < 0 || strip_id > 255)
        return false;   //< error invalid strip ID

      // read tags in correct order, skip wrong ids first
      if ((u_int8_t)strip_id != current_strip_id)
      {
        strip = strip->NextSiblingElement(XML_STRIP_ELEMENT);
        continue;
      }


      //************ READ STRIP *************/
      TrajectoryPtr driving_strip = std::make_shared<Trajectory2d>();
      driving_strip->isForwardTrajectory() = forward;

      // iterate over points of strip
      for (XMLElement* p_child = strip->FirstChildElement(XML_POINT_ELEMENT); p_child != nullptr; p_child = p_child->NextSiblingElement(XML_POINT_ELEMENT))
      {
        ExtendedPose2d p;
        PoseTraits<Pose2d>::fromPositionAndOrientationRPY(p.pose(),
                                                          p_child->DoubleAttribute(XML_POINT_X),
                                                          p_child->DoubleAttribute(XML_POINT_Y),
                                                          p_child->DoubleAttribute(XML_POINT_YAW));
        driving_strip->push_back(p);
      }

      if (driving_strip->empty())   //< this is an error (empty file?)
        return false;

      // save strip as strip of current diversion
      current_diversion.push_back(driving_strip);
      // save strip in complete strip container
      m_strip_collection->push_back(driving_strip);

      // prepare search for next strip_id -> begin searching again at first strip
      current_strip_id++;
      strip = diversion->FirstChildElement(XML_STRIP_ELEMENT);
    }

    if (current_diversion.empty())
      return false;

    // Save strips of this diversion
    m_diversion_collection->push_back(current_diversion);

    //*********** READ ENDPOSE *************/
    XMLElement* p_endpose = diversion->FirstChildElement(XML_ENDPOSE_ELEMENT);
    if (p_endpose == nullptr)
      return false;

    Pose2d endpose;
    PoseTraits<Pose2d>::fromPositionAndOrientationRPY(endpose,
                           p_endpose->DoubleAttribute(XML_ENDPOSE_X),
                           p_endpose->DoubleAttribute(XML_ENDPOSE_Y),
                           p_endpose->DoubleAttribute(XML_ENDPOSE_THETA));
    m_endpose_collection->push_back(endpose);


    // prepare search for next diversion id
    current_diversion_id++;
    diversion = titleElement->FirstChildElement(XML_DIVERSION_ELEMENT);
  }

  if (current_diversion_id == 0)
    return false;
  m_number_of_diversions = current_diversion_id;


  // check for correct number of diversions
  assert(m_number_of_diversions == m_endpose_collection->size());


  //************ READ PATCH CORNERS *********/
  // create corner array
  m_patch_boundary = std::make_shared<PatchBoundary>();

  for (XMLElement* corner = titleElement->FirstChildElement(XML_CORNER_ELEMENT); corner != nullptr; corner = corner->NextSiblingElement(XML_CORNER_ELEMENT))
  {
    std::string current_corner_id = corner->Attribute(XML_CORNER_ID);

    u_int8_t i;
    for (i = 0; i < (u_int8_t)CORNER_POINT::CORNER_POINT_COUNT; i++)
    {
      if (current_corner_id == XML_CORNER_IDS[i])
        break;
    }
    if (i == (u_int8_t)CORNER_POINT::CORNER_POINT_COUNT) //< error wrong corner id
      return false;

    (*m_patch_boundary)[i] = Position2d(corner->DoubleAttribute(XML_CORNER_XVALUE), corner->DoubleAttribute(XML_CORNER_YVALUE));
  }

  return true;
}

} //ns
