// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-07
 *
 */
//----------------------------------------------------------------------

#include "RoadJunction.h"

namespace katana
{

RoadPatch::TrajectoryPtr RoadJunction::getDrivingStrip(RoadBase::DrivingStripId drivingStrip) const
{
  DiversionId diversion = 0;
  if(m_direction == Action::LEFT) {
    diversion = 0;
  } else if(m_direction == Action::RIGHT) {
    diversion = 2;
  } else if(m_direction == Action::STRAIGHT) {
    diversion = 1;
  } else {
    assert(false && "Unkown direction passed.");
  }
  #ifdef KATANA_MC_MANEUVER_DEBUG
    std::cout << "MC Maneuver; Diversion of junction with id " << this->getId() << " " << this->getAnchorPose() << " is " << (int32_t) diversion << std::endl;
  #endif
  return katana::RoadBase::getDrivingStrip(drivingStrip, diversion);
}
 
 
void RoadJunction::setRightOfWay(const RightOfWayDirection& right_of_way)
{
  if (right_of_way.first == right_of_way.second && right_of_way.first != RIGHT_OF_WAY::PRIORITY_FROM_RIGHT)   //< error
    m_right_of_way = RightOfWayDirection(RIGHT_OF_WAY::UNDEFINED, RIGHT_OF_WAY::UNDEFINED);
  else
    m_right_of_way = right_of_way;

  if (m_right_of_way.first == RIGHT_OF_WAY::PRIORITY_FROM_RIGHT || m_right_of_way.first == RIGHT_OF_WAY::OWN)
    return;

  if (m_right_of_way.second == RIGHT_OF_WAY::PRIORITY_FROM_RIGHT)
    std::swap(m_right_of_way.first, m_right_of_way.second);
  else if (m_right_of_way.second == RIGHT_OF_WAY::OWN)
    std::swap(m_right_of_way.first, m_right_of_way.second);
  else if (m_right_of_way.second == RIGHT_OF_WAY::RIGHT)
    std::swap(m_right_of_way.first, m_right_of_way.second);
  else if (m_right_of_way.first == RIGHT_OF_WAY::LEFT)
    std::swap(m_right_of_way.first, m_right_of_way.second);
}

void RoadJunction::setRightOfWay(TrafficSign ts)
{
  m_is_stop = false;

  switch(ts)
  {
  case TrafficSign::JUNCTION_STOP_GIVE_WAY:
    m_is_stop = true;
  case TrafficSign::JUNCTION_GIVE_WAY:
    m_right_of_way = RightOfWayDirection(RIGHT_OF_WAY::RIGHT, RIGHT_OF_WAY::LEFT);
    break;
  case TrafficSign::JUNCTION_PRIORITY:
    m_right_of_way = RightOfWayDirection(RIGHT_OF_WAY::OWN, RIGHT_OF_WAY::STRAIGHT);
    break;
  case TrafficSign::JUNCTION_PRIORITY_FROM_RIGHT:
    m_right_of_way = RightOfWayDirection(RIGHT_OF_WAY::PRIORITY_FROM_RIGHT, RIGHT_OF_WAY::UNDEFINED);
    break;
  default:
    m_right_of_way = RightOfWayDirection(RIGHT_OF_WAY::UNDEFINED, RIGHT_OF_WAY::UNDEFINED);
    m_is_stop = true;
    break;
  }
}

std::vector<DIVERSION> RoadJunction::yieldRightOfWayTo(DIVERSION where_to_go) const
{
  std::vector<DIVERSION> yield_to;

  if (where_to_go == DIVERSION::UNDEFINED)    //< error
  {
    yield_to.push_back(DIVERSION::RIGHT);
    yield_to.push_back(DIVERSION::STRAIGHT);
    yield_to.push_back(DIVERSION::LEFT);
    return yield_to;
  }

  if (m_right_of_way.first == RIGHT_OF_WAY::OWN)
  {
    if (m_right_of_way.second == RIGHT_OF_WAY::STRAIGHT && where_to_go == DIVERSION::LEFT)
    {
      yield_to.push_back(DIVERSION::STRAIGHT);
    }
    else if (m_right_of_way.second == RIGHT_OF_WAY::RIGHT && where_to_go == DIVERSION::STRAIGHT)
    {
      yield_to.push_back(DIVERSION::RIGHT);
    }
  }
  else if (m_right_of_way.first == RIGHT_OF_WAY::PRIORITY_FROM_RIGHT)
  {
    if (where_to_go != DIVERSION::RIGHT)
    {
      yield_to.push_back(DIVERSION::RIGHT);
      if (where_to_go == DIVERSION::LEFT)
      {
        yield_to.push_back(DIVERSION::STRAIGHT);
      }
    }
  }
  else if (m_right_of_way.first == RIGHT_OF_WAY::UNDEFINED || m_right_of_way.second == RIGHT_OF_WAY::UNDEFINED)
  {
    yield_to.push_back(DIVERSION::RIGHT);
    yield_to.push_back(DIVERSION::STRAIGHT);
    yield_to.push_back(DIVERSION::LEFT);
  }
  else
  {
    if (m_right_of_way.first == RIGHT_OF_WAY::RIGHT)
    {
      // right - left
      if (m_right_of_way.second == RIGHT_OF_WAY::LEFT)
      {
        yield_to.push_back(DIVERSION::LEFT);
        if (where_to_go != DIVERSION::RIGHT)
          yield_to.push_back(DIVERSION::RIGHT);
        if (where_to_go == DIVERSION::LEFT)
          yield_to.push_back(DIVERSION::STRAIGHT);
      }
      // right - straight
      else
      {
        yield_to.push_back(DIVERSION::STRAIGHT);
        if (where_to_go != DIVERSION::RIGHT)
          yield_to.push_back(DIVERSION::RIGHT);
      }
    }
    // straight - left
    else
    {
      yield_to.push_back(DIVERSION::STRAIGHT);
      yield_to.push_back(DIVERSION::LEFT);
      if (where_to_go != DIVERSION::RIGHT)
      {
        yield_to.push_back(DIVERSION::RIGHT);
      }
    }
  }
  return yield_to;
}

} //ns
