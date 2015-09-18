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

#include "ParkingAssistant.h"

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
#include <iostream>
#endif

using namespace oadrive::core;

namespace katana
{

const double ParkingAssistant::PARKING_SPACE_LENGTH[PARKING_SPACE_TYPE_COUNT] = { 0.48, 0.765 };
const double ParkingAssistant::PARKING_SPACE_DEPTH_DIFF[PARKING_SPACE_TYPE_COUNT] = { 0.45, 0.21 };
const double ParkingAssistant::PARKING_SPACE_DEPTH_CENTER[PARKING_SPACE_TYPE_COUNT] = { 0.625, 0.50 };

const double ParkingAssistant::THRESHOLD_CORR_TAKEN[PARKING_SPACE_TYPE_COUNT] = {-0.05, 0.01};
const double ParkingAssistant::THRESHOLD_CORR_EMPTY[PARKING_SPACE_TYPE_COUNT] = {0.08, 0.03};

void ParkingAssistant::enableSearching(ParkingSpace parking_space, const double& start_distance, SideOfVehicle side, u_int32_t search_empty)
{
  m_spot_found = false;
  m_type = parking_space;
  m_start_distance = start_distance;
  m_search_empty = search_empty;
  m_side_of_vehicle = side;
  m_wait_for_rear = true;
  m_occupancy_grid.clear();
  m_evaluated_occupancy_grid.clear();

  // calculate values for side to look for
  for(u_int8_t i = 0; i < (u_int8_t)PARKING_SPACE_TYPE_COUNT; i++)
  {
    m_parking_space_depth_diff[i] = m_side_of_vehicle * PARKING_SPACE_DEPTH_DIFF[i];
    m_parking_space_depth_center[i] = m_side_of_vehicle * PARKING_SPACE_DEPTH_CENTER[i];
  }

  const std::size_t parking_space_size = PARKING_SPACE_LENGTH[m_type] / SECTOR_RESOLUTION;
  const std::size_t parking_vehicle_size = PARKING_VEHICLE_LENGTH / SECTOR_RESOLUTION;

  assert(parking_space_size > 4 && "Parking space resolution to small");
  assert(parking_vehicle_size > 3 && "Parking space resolution to small");

  // create vector to find empty/taken spot
  m_evalution_vector_empty.resize(parking_space_size, m_parking_space_depth_diff[m_type]);
  m_evalution_vector_taken.resize(parking_vehicle_size, -m_parking_space_depth_diff[m_type]);

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
  std::cout <<"[ParkingAssistant] searching for " <<(parking_space == ParkingSpace::CROSS ? "cross" : "parallel") <<" parking spot." <<std::endl;
#endif
}

void ParkingAssistant::addSignificantObstacle(const Obstacle::ConstPtr& obstacle, const Pose2d& vehicle_pose, const double& driven_distance)
{
  // obtain vehicle coordinates
  const Pose2d obstacle_vehicle = vehicle_pose.inverse() * obstacle->getPose();

  const double long_position = driven_distance - m_start_distance + obstacle_vehicle.translation().x();

  if (long_position < 0.0)    // obstacle before beginning of range, discard
    return;

  // assign to slot
  u_int32_t slot = long_position / SECTOR_RESOLUTION;

  if (obstacle_vehicle.translation().y() > 0.0)   // wrong side of vehicle, discard
    return;

  // expand grid if necessary
  while (slot >=m_occupancy_grid.size())
  {
    m_occupancy_grid.push_back(Measurements());
  }
  m_occupancy_grid[slot].push_back(obstacle_vehicle.translation().y()-m_parking_space_depth_center[m_type]);

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
  std::cout <<"[ParkingAssistant] Added measurement " <<obstacle_vehicle.translation().y() <<" to slot " <<slot <<"." <<std::endl;
#endif

}

bool ParkingAssistant::foundAppropriateSpot(double& at_distance) const
{
  if (!m_spot_found)
    return false;

  at_distance = m_start_distance + SECTOR_RESOLUTION * m_start_of_empty_spot + SECTOR_RESOLUTION*0.5;

  return true;
}

void ParkingAssistant::evaluateMeasurments(std::size_t from, std::size_t to, const double& driven_distance)
{
#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
  std::cout <<"[ParkingAssistant] Evaluating " <<std::endl;
#endif

  m_evaluated_occupancy_grid.resize(to, 0.0);

  for (std::size_t i = from; i < to; i++)
  {
    if (m_occupancy_grid[i].empty())
    {
      m_evaluated_occupancy_grid[i] = m_parking_space_depth_diff[m_type];
    }
    else
    {
      double average = 0.0;
      for (std::size_t v = 0; v < m_occupancy_grid[i].size(); v++)
      {
        average += m_occupancy_grid[i][v];
      }
      average /= m_occupancy_grid[i].size();
      if (average < m_parking_space_depth_diff[m_type])
        average = m_parking_space_depth_diff[m_type];

      m_evaluated_occupancy_grid[i] = average;
    }
  }

  //!If no sensor data arrives, the size of the occupancy grid is increased
  if(to < (driven_distance-m_start_distance)/SECTOR_RESOLUTION)
  {
    m_evaluated_occupancy_grid.resize((driven_distance-m_start_distance)/SECTOR_RESOLUTION);
    std::fill(m_evaluated_occupancy_grid.begin()+to, m_evaluated_occupancy_grid.end(),m_parking_space_depth_diff[m_type]);
  }

  //////////////////////////// TAKEN SPOT /////////////////////////////
  std::size_t pos_taken = m_search_empty;
  double value;
  // If m_search_only_empty is false, vehicle first needs to pass the first taken spot
  if (m_search_empty == 0)
  {
    // not enough seen to find spots
    if (m_evaluated_occupancy_grid.size() < m_evalution_vector_empty.size())
      return;

  #ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
    printEvaluatedMeasurementGrid();
  #endif

    // search for occupied parking spot
    pos_taken = correlateEvaluation(m_evalution_vector_taken, value, THRESHOLD_CORR_TAKEN[m_type]);

    if (value <= THRESHOLD_CORR_TAKEN[m_type])    //< not good enough
    {
      return;
    }

  #ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
    std::cout <<"[ParkingAssistant] First taken spot found at slot: " <<pos_taken <<std::endl;
  #endif
  }

  // not enough seen to find empty spot
  if (pos_taken + m_evalution_vector_empty.size() > m_evaluated_occupancy_grid.size())
    return;


  if (m_wait_for_rear && ((driven_distance - m_start_distance) / SECTOR_RESOLUTION) < pos_taken + m_evalution_vector_empty.size())
  {
    return;
  }

  // search for empty parking spot
  std::size_t pos_empty = correlateEvaluation(pos_taken, m_evaluated_occupancy_grid.size(), m_evalution_vector_empty, value, THRESHOLD_CORR_EMPTY[m_type]);

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
  std::cout <<"[ParkingAssistant] Max correlation value empty: " <<value <<" found at slot: " <<pos_empty <<std::endl;
#endif

  if (value <= THRESHOLD_CORR_EMPTY[m_type])    //< not good enough
  {
    return;
  }

  // SPOT FOUND
  m_spot_found = true;
  m_start_of_empty_spot = pos_empty;
}

std::size_t ParkingAssistant::correlateEvaluation(std::size_t from, std::size_t to, const EvaluatedOccupancyGrid& eval, double& max_value, const double& stop_threshold)
{
  assert(!eval.empty() && "!eval.empty()");
  assert(to - from - eval.size() >= 0 && "to - from - eval.size() >= 0");

  const std::size_t diff = to - from - eval.size() + 1;

  max_value = -std::numeric_limits<double>::infinity();
  std::size_t max = 0;

  for (std::size_t shift = from; shift < diff+from; shift++)
  {
    double value = 0.0;

    for (std::size_t i = 0; i < eval.size(); i++)
    {
      value += eval[i]*m_evaluated_occupancy_grid[i+shift];
    }

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
    std::cout <<"korr: " <<value <<std::endl;
#endif

    const double norm = value / eval.size();
    if (norm > stop_threshold)
    {
      max_value = norm;
      return shift;
    }

    if (value > max_value)
    {
      max_value = value;
      max = shift;
    }
  }

  max_value /= eval.size();

  return max;
}

#ifdef KATANA_MC_PARKING_ASSISTANT_DEBUG
void ParkingAssistant::printEvaluatedMeasurementGrid() const
{
  const bool print_values = m_evaluated_occupancy_grid.size() == m_occupancy_grid.size();

  std::cout <<std::endl;

  for (std::size_t i = 0; i < m_evaluated_occupancy_grid.size(); i++)
  {
    u_int32_t value = -(m_evaluated_occupancy_grid[i] + m_parking_space_depth_center[m_type]) * 10.0;
    while (value > 0)
    {
      std::cout <<"x";
      value--;
    }
    if (i < m_occupancy_grid.size())
    {
      for (std::size_t v = 0; v < m_occupancy_grid[i].size(); v++)
      {
        std::cout <<"  " <<m_occupancy_grid[i][v] + m_parking_space_depth_center[m_type];
      }
    }

    std::cout <<std::endl;
  }
}
#endif

} // ns
