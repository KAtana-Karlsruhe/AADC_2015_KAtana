// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-15
 *
 */
//----------------------------------------------------------------------

#ifndef _MISSION_CONTROL_PARKING_ASSISTANT_H
#define _MISSION_CONTROL_PARKING_ASSISTANT_H

#include "katanaCommon/katanaCommon.h"
#include <oadrive_core/Pose.h>

#include "Obstacle.h"

namespace katana
{

enum ParkingSpace : u_int8_t
{
  CROSS=0,
  PARALLEL=1,
  PARKING_SPACE_TYPE_COUNT=2
};

enum SideOfVehicle : int8_t
{
  RIGHT=-1,
  LEFT=1
};

class ParkingAssistant
{
public:

  friend class World;

  static constexpr double SECTOR_RESOLUTION = 0.05;
  static constexpr double PARKING_VEHICLE_LENGTH = 0.3;
  static const double THRESHOLD_CORR_TAKEN[];
  static const double THRESHOLD_CORR_EMPTY[];

  static const double PARKING_SPACE_LENGTH[];
  static const double PARKING_SPACE_DEPTH_DIFF[];
  static const double PARKING_SPACE_DEPTH_CENTER[];

  //! Map to search for empty parking spot
  typedef std::vector<double> Measurements;
  typedef std::vector<Measurements> OccupancyGrid;
  typedef std::vector<double> EvaluatedOccupancyGrid;

  //! Convenience pointer
  typedef std::shared_ptr<ParkingAssistant> Ptr;
  typedef std::shared_ptr<const ParkingAssistant> ConstPtr;

  //! Constructor
  ParkingAssistant()
    : m_type(ParkingSpace::PARKING_SPACE_TYPE_COUNT)
  {

  }

  //! Destructor
  virtual ~ParkingAssistant()     {}

  //! Return position of first appropriate parking spot
  bool foundAppropriateSpot(double& at_distance) const;

  //! Read Access
  const double& getStartDistance() const    { return m_start_distance; }
  ParkingSpace getParkingType() const       { return m_type; }

  //!
  void setWaitForRear(bool wait_for_rear)   { m_wait_for_rear = wait_for_rear; }

  //! Check for parking spot
  void evaluateMeasurments(const double& driven_distance)  { evaluateMeasurments(0, m_occupancy_grid.size(), driven_distance);}

private:

  //!
  void enableSearching(ParkingSpace parking_space, const double& start_distance, SideOfVehicle side = RIGHT, u_int32_t search_empty = 0);

  //!
  void addSignificantObstacle(const Obstacle::ConstPtr& obstacle, const oadrive::core::Pose2d& vehicle_pose, const double& driven_distance);

  //!
  void evaluateMeasurments(std::size_t from, std::size_t to, const double& driven_distance);

  //! Correlation, return max
  std::size_t correlateEvaluation(std::size_t from, std::size_t to, const EvaluatedOccupancyGrid& eval, double& max_value, const double& stop_threshold = std::numeric_limits<double>::infinity());
  std::size_t correlateEvaluation(const EvaluatedOccupancyGrid& eval, double& max_value, const double& stop_threshold = std::numeric_limits<double>::infinity())
  {
    return correlateEvaluation(0, m_evaluated_occupancy_grid.size(), eval, max_value, stop_threshold);
  }

  //! Flag indicating currently searched parking spot
  ParkingSpace m_type;

  //! record obstacles when searching for parking spot
  OccupancyGrid m_occupancy_grid;

  //! Longitudinal start position
  double m_start_distance;

  //! Represents an empty spot
  EvaluatedOccupancyGrid m_evaluated_occupancy_grid;

  //! Size until evalutation
  u_int32_t m_refresh_evalution_size;

  //! Correlation vector to find empty/taken spot
  EvaluatedOccupancyGrid m_evalution_vector_empty;
  EvaluatedOccupancyGrid m_evalution_vector_taken;

  //! Flag if assistant only searches for first empty spot
  u_int32_t m_search_empty;

  //! Which side are we looking for?
  SideOfVehicle m_side_of_vehicle;

  //! Flag if spot found
  bool m_spot_found;
  std::size_t m_start_of_empty_spot;

  //! If set to true, do not wait for rear sensors
  bool m_wait_for_rear;


  double m_parking_space_depth_diff[PARKING_SPACE_TYPE_COUNT];
  double m_parking_space_depth_center[PARKING_SPACE_TYPE_COUNT];


#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
  //! print to console
  void printEvaluatedMeasurementGrid() const;
#endif
};


} // ns

#endif //_MISSION_CONTROL_PARKING_ASSISTANT_H
