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

#ifndef _MISSION_CONTROL_MANEUVER_H_
#define _MISSION_CONTROL_MANEUVER_H_

#include "tinyxml2.h"

#include <string>
#include <vector>
#include <iostream>
#include <queue>

#include "katanaCommon/katanaCommon.h"

namespace katana
{

class Maneuver
{
public:

  typedef u_int32_t _maneuver_id_type;
  typedef u_int32_t _sector_id_type;

  typedef std::pair<Action, _maneuver_id_type> AADCManeuver;
  typedef std::pair<std::vector<AADCManeuver>, _sector_id_type> Sector;
  typedef std::vector<Sector> SectorContainer;

  //! Pointer shorthand
  typedef std::shared_ptr<Maneuver> Ptr;
  typedef std::shared_ptr<const Maneuver> ConstPtr;

  //! Constructor
  Maneuver();

  //! Destructor
  virtual ~Maneuver()   {}

  //! Read route file
  void readManeuver(const std::string& file);

  //! Ready?
  bool isReady() const      { return m_ready; }

  //! Returns the current maneuver action
  Action getCurrentManeuver() const
  {
    if (!m_ready)
      return Action::UNKNOWN;

    if(!m_external_actions.empty()) {
      #ifdef KATANA_MC_MANEUVER_DEBUG
	std::cout << "MC maneuver: returning current EXTERNAL maneuver: " << m_external_actions.front() << std::endl;
      #endif
      return m_external_actions.front();
    }

    #ifdef KATANA_MC_MANEUVER_DEBUG
      std::cout << "MC maneuver: returning current maneuver: " << m_data.at(m_current_sector).first.at(m_current_maneuver).second << std::endl;
    #endif

    return m_data.at(m_current_sector).first.at(m_current_maneuver).first;
  }

  //! Returns the id of the current maneuver; UINT32_MAX if not maneuver file not fully read
  _maneuver_id_type getCurrentManeuverId() const
  {
    if (!m_ready)
      return UINT32_MAX;

    if(!m_external_actions.empty())
      return UINT32_MAX;

    return m_data.at(m_current_sector).first.at(m_current_maneuver).second;
  }

  //! True if current maneuver is last in its sector
  bool isLastManeuverInSector() const
  {
    return m_data.at(m_current_sector).first.size() == (m_current_maneuver + 1);
  }

  //! True if currently in the last sector
  bool isLastSector() const
  {
    return m_data.size() == (m_current_sector + 1);
  }

  //! True if last maneuver of list
  bool isFinished() const       { return isLastSector() && isLastManeuverInSector(); }

  //! Jump to next maneuver, true if there is a maneuver left in the current sector, otherwise false
  bool advanceManeuver();

  //! Jump to next sector, false if last sector is already reached
  bool nextSector();

  //! Jump to first maneuver of current sector
  void resetSector();

  //! Jump to first maneuver of first sector
  void resetPosition();

  //! Jump to Manveuver with given id (set sectorid and maneuver in sector)
  void setManeuver(_maneuver_id_type maneuver_id);

  //! Strings representing the maneuvers
  static const char* XML_ACTION_STRINGS[];

  //! Constant strings for XMLElements
  static const constexpr char* XML_AADC_LIST = "AADC-Maneuver-List";
  static const constexpr char* XML_AADC_SECTOR = "AADC-Sector";
  static const constexpr char* XML_AADC_MANEUVER = "AADC-Maneuver";
  static const constexpr char* XML_ATTRIBUTE_ID = "id";
  static const constexpr char* XML_ATTRIBUTE_ACTION = "action";

  //! False if maneuver list is finished, otherwise true, automaticly jumps to next sector
  bool increaseManeuver();

  //! Reset the queue of external actions
  void resetExternalTrigger() { m_external_actions = {}; }

  void setExternalTrigger(Action externalAction)	{ m_external_actions.push(externalAction); }

private:
  //! Helper: find action from string
  Action getActionFromXMLString(const char* str);

  //! Helper: iterate through actions
  Action next( const Action& a) { return (Action)(((std::underlying_type<Action>::type)(a) + 1)); }

  //! Data container
  SectorContainer m_data;

  //! Ready after reading of route
  bool m_ready;

  //! Current sector
  u_int32_t m_current_sector;

  //! Current maneuver
  u_int32_t m_current_maneuver;

  std::queue<Action> m_external_actions;
};

} //ns


#endif //_MISSION_CONTROL_MANEUVER_H_
