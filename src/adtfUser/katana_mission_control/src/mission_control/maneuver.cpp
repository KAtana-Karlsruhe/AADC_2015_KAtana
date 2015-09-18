// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-11-29
 *
 */
//----------------------------------------------------------------------

#include "mission_control/maneuver.h"

#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
#include <iostream>
#endif

using namespace tinyxml2;

namespace katana
{

const char* Maneuver::XML_ACTION_STRINGS[] =
{
  "left",
  "straight",
  "right",
  "parallel_parking",
  "cross_parking",
  "pull_out_left",
  "pull_out_right"
};

Maneuver::Maneuver()
  : m_ready(false)
  , m_current_sector(0)
  , m_current_maneuver(0)
{
}

void Maneuver::readManeuver(const std::string& file)
{
  XMLDocument doc;
  doc.LoadFile(file.c_str());

  XMLElement* titleElement = doc.FirstChildElement(XML_AADC_LIST);

  _maneuver_id_type maneuver_id_counter = 0;

  for (XMLElement* sector = titleElement->FirstChildElement(XML_AADC_SECTOR); sector != nullptr; sector = sector->NextSiblingElement(XML_AADC_SECTOR))
  {
    //Add this new sector
    m_data.push_back(Sector(std::vector<AADCManeuver>(), sector->IntAttribute(XML_ATTRIBUTE_ID)));

    //Read maneuver
    for (XMLElement* m = sector->FirstChildElement(XML_AADC_MANEUVER); m != nullptr; m = m->NextSiblingElement(XML_AADC_MANEUVER))
    {
      AADCManeuver element(getActionFromXMLString(m->Attribute(XML_ATTRIBUTE_ACTION)), m->IntAttribute(XML_ATTRIBUTE_ID));
      if (element.first == Action::UNKNOWN)
      {
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
        std::cout <<"WARNING: Unknown maneuver list entry " <<m->Attribute(XML_ATTRIBUTE_ACTION) <<" - skipping" <<std::endl;
#endif
        ++maneuver_id_counter;
        continue;
      }
      // add new maneuver
      m_data.back().first.push_back(element);

#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
      if (element.second != maneuver_id_counter)
        std::cout <<"WARNING: IDs of maneuver entries not in ascending order (at id " <<element.second <<"). This may result in undefined conditions!" <<std::endl;
#endif

      // increase id counter
      ++maneuver_id_counter;
    }

    //if empty sector -> remove sector
    if (m_data.back().first.empty())
      m_data.erase(m_data.end() - 1);
  }

  resetPosition();
  m_ready = true;
}

Action Maneuver::getActionFromXMLString(const char *str)
{
  Action i;
  for (i = Action::First; i != Action::UNKNOWN; i = next(i) )
  {
     if (strcmp(XML_ACTION_STRINGS[(int8_t)i], str) == 0)
       break;
  }
  return i;
}

void Maneuver::resetPosition()
{
  #ifdef KATANA_MC_MANEUVER_DEBUG
    std::cout << "MC maneuver: Resetting position." << std::endl;
  #endif
  m_current_maneuver = 0;
  m_current_sector = 0;
}

void Maneuver::resetSector()
{
  #ifdef KATANA_MC_MANEUVER_DEBUG
    std::cout << "MC maneuver: Resetting sector." << std::endl;
  #endif
  m_current_maneuver = 0;
}

bool Maneuver::advanceManeuver()
{
  #ifdef KATANA_MC_MANEUVER_DEBUG
    std::cout << "MC maneuver: Going to next maneuver." << std::endl;
  #endif
  if (isLastManeuverInSector())
    return false;

  ++m_current_maneuver;
  return true;
}

bool Maneuver::nextSector()
{
  if (isLastSector())
    return false;

  #ifdef KATANA_MC_MANEUVER_DEBUG
    std::cout << "MC maneuver: Going to next sector." << std::endl;
  #endif
  m_current_maneuver = 0;
  ++m_current_sector;

  return true;
}

void Maneuver::setManeuver(Maneuver::_maneuver_id_type maneuver_id)
{
  for(uint32_t sectorId = 0; sectorId < m_data.size(); ++sectorId) {
    for(uint32_t i = 0; i < m_data.at(sectorId).first.size(); ++i) {
      if(m_data.at(sectorId).first.at(i).second == maneuver_id) {
	m_current_sector = sectorId;
	m_current_maneuver = i;
	#ifdef KATANA_MC_MANEUVER_DEBUG
	  std::cout << "MC maneuver: set maneuver to " <<m_data.at(sectorId).first.at(i).second << " (position " << m_current_maneuver << ") and sector to " << m_current_sector << std::endl;
	#endif
	return;
      }
    }
  }
}

bool Maneuver::increaseManeuver()
{
  // If external actions are available use them
  if(!m_external_actions.empty()) {
    m_external_actions.pop();
    return true;
  }

  if (!advanceManeuver())
    return nextSector();
  return true;
}

} // ns
