// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-02
 *
 */
//----------------------------------------------------------------------

#include "mission_control/RoadSection.h"

namespace katana
{

u_int32_t RoadSection::evaluation(const Pose& p)
{
  // kind elliptic distance function... TODO: improve this function!
  _position_type distance = Point(p.getX(), p.getY() *2).abs();

  if (distance > 0.3)
    return 0;

  return 0.4 - distance;
}

RoadSection::RoadSection(const RoadBase::Ptr& patch)
  : m_patch(patch)
{
  m_port_number = m_patch->getEndPoseContainer().size() + 1;
  m_sections.resize(m_port_number, Ptr());
}

RoadSection::SectionContainer RoadSection::getAdjacent(const Ptr& from) const
{
  // create copy
  SectionContainer ret = m_sections;

  // remove every element == from
  ret.erase(std::remove(ret.begin(), ret.end(), from), ret.end());

  // remove every nullptr
  ret.erase(std::remove(ret.begin(), ret.end(), nullptr), ret.end());

  return ret;
}

} // ns
