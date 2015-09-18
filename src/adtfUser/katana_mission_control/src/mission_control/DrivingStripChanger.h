// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-19
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_DRIVINGSTRIPCHANGER_H
#define _MISSION_CONTROL_DRIVINGSTRIPCHANGER_H

#include "katanaCommon/katanaCommon.h"
#include "Pose.h"
#include "System.h"

#include <chrono>

namespace katana
{



class DrivingStripChanger
{
public:
  static constexpr double DISTANCE_TO_CHECK_LANE = 0.1;

  //! Convenience pointer
  typedef std::shared_ptr<DrivingStripChanger> Ptr;

  //! Constructor
  DrivingStripChanger() = delete;
  DrivingStripChanger(System::Ptr system)
    : m_system(system)
    , m_overtaking(false)
    , m_reversing(false)
    , m_changing_lane(false)
  {

  }

  //! No copying, instance handles jobs for one lanetracker
  DrivingStripChanger(const DrivingStripChanger& rs) = delete;
  DrivingStripChanger& operator=(const DrivingStripChanger& rhs) = delete;

  //! Read Access
  bool isCurrentlyOvertaking() const    { return m_overtaking; }
  bool isCurrentlyLaneChanging() const  { return m_changing_lane; }
  bool isCurrentylReversing() const     { return m_reversing; }

  //! Call in timer function to check if lane is empty again/ or reversed
  void timerCheck();

  //! Triggers change to another driving strip
  void changeLane(bool change_back_automaticly = false);

  //! Check if needing to reverse
  double needToReverse() const;

  //! Take current trajectory and drive in reverse
  void driveReverse(double dist, bool change_back_automaticly);

private:

  //! Access to modules
  System::Ptr m_system;

  //! Flag if currently in overtaking mode
  bool m_overtaking;

  //! Flag if currently in reversing mode
  bool m_reversing;
  bool m_change_back_automaticly;

  //! Flag if currently changing lane
  bool m_changing_lane;

  //! Parking assistant checks for empty lane
  ParkingAssistant::Ptr m_parking_assistant;

  //! Double to trigger evaluating in parking assistant
  double m_driven_distance_checked;
};


} // ns

#endif //_MISSION_CONTROL_DRIVINGSTRIPCHANGER_H
