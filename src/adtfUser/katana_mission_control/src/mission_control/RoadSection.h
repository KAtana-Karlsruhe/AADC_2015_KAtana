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

#ifndef _MISSION_CONTROL_ROAD_SECTION_H
#define _MISSION_CONTROL_ROAD_SECTION_H

#include "katanaCommon/katanaCommon.h"
#include "RoadBase.h"
#include "Pose.h"


namespace katana
{

class RoadSection
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<RoadSection> Ptr;
  typedef std::shared_ptr<const RoadSection> ConstPtr;
  typedef std::vector<Ptr> SectionContainer;

  //! PatchType
  typedef RoadBase::Ptr Patch;

  //! Constructor
  RoadSection() = delete;
  RoadSection(const RoadBase::Ptr& patch);

  //! No copying
  RoadSection& operator=(const RoadSection& rhs) = delete;

  //! Destructor
  virtual ~RoadSection()      {}

  //! Get number of ports
  u_int8_t getPortNumber() const    { return m_port_number; }

  //! Get adjacent RoadSections, not containing <from>
  SectionContainer getAdjacent(const Ptr& from = nullptr) const;

protected:

  //! Number of ports
  u_int8_t m_port_number;

  //! Pointer to bordering sections
  SectionContainer m_sections;

  //! Pointer to patch
  Patch m_patch;

  //! Function to check fitting of patches
  static u_int32_t evaluation(const Pose& p);

};


} // ns

#endif //_MISSION_CONTROL_ROAD_SECTION_H
