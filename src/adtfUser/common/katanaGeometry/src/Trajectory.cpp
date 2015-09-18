// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-01-18
 *
 */
//----------------------------------------------------------------------

#include "Trajectory.h"

#include <algorithm>

namespace katana
{

Trajectory::Trajectory()
{
  m_trajectory = std::make_shared<Trajectory::PointContainer>();
  m_length = 0.0;
}
Trajectory::Trajectory(const Trajectory& t)
{
  // make a copy of the point container
  m_trajectory = std::make_shared<PointContainer>(*t.m_trajectory);

  // standard stuff...
  m_distances = t.m_distances;
  m_length = t.m_length;
}

Trajectory& Trajectory::operator =(const Trajectory& t)
{
  // make a copy of the point container
  m_trajectory = std::make_shared<PointContainer>(*t.m_trajectory);

  // standard stuff...
  m_distances = t.m_distances;
  m_length = t.m_length;

  return *this;
}

Trajectory::~Trajectory()
{

}

void Trajectory::transformToWorld(const Pose& base)
{
  double sin_a = sin(base.getTheta());
  double cos_a = cos(base.getTheta());

  Pose result;

  // iterate over points
  for (Point& i : *m_trajectory)
  {
    // Rotation
    result.setX(i.getX()*cos_a - i.getY()*sin_a);
    result.setY(i.getX()*sin_a + i.getY()*cos_a);

    // Translation
    (Point)result.operator+=(base);

    // Save
    i = result;
  }
}

void Trajectory::append(const Trajectory& t)
{
  // append points from other trajectory
  m_trajectory->insert(m_trajectory->end(), t.getTrajectory().begin(), t.getTrajectory().end());
}

void Trajectory::append(const Point& p)
{
  m_trajectory->push_back(p);
}

void Trajectory::append(const PointContainer& points)
{
  m_trajectory->insert(m_trajectory->end(), points.begin(), points.end());
}

void Trajectory::insertAhead(const Trajectory& t)
{
  m_trajectory->insert(m_trajectory->begin(), t.getTrajectory().begin(), t.getTrajectory().end());
}


void Trajectory::createFromSubTrajectories(const std::vector<Trajectory::ConstPtr>& trajectories)
{
  // clear existing points
  m_trajectory->clear();

  // we need interpolated trajectories which can be connected at the end
  std::vector<Trajectory> interpolated;

  // make working copies if non-empty
  for (const Trajectory::ConstPtr& i : trajectories)
  {
    if (!i->empty())
      interpolated.push_back(*i);
  }

  // adjust every trajectory to the beginning of the next one
  for (u_int32_t i = 0; i < interpolated.size() - 1; i++)
  {
    interpolated.at(i).adjustTrajectory(interpolated.at(i+1).getTrajectory().front());
  }

  // join the trajectories
  for (const Trajectory& i : interpolated)
  {
    append(i);
  }
}

void Trajectory::adjustTrajectory(const Point& target)
{
  assert(!m_trajectory->empty());

  // diff
  Point diff = target - m_trajectory->back();

  // m_distances and overall length will be available
  updateDistances();

  assert(m_trajectory->size() == m_distances.size()+1); // check array size

  double length_ratio = 0.0;
  for (u_int32_t i = 1; i < m_trajectory->size() - 1; i++)
  {
    length_ratio += m_distances.at(i-1);
    m_trajectory->at(i) += diff * (length_ratio / m_length);
  }

  // set last point exactly on target
  m_trajectory->back() = target;
}

void Trajectory::updateDistances()
{
  m_distances.clear();

  for (u_int32_t i = 1; i < m_trajectory->size(); i++)
  {
    m_distances.push_back(Point(m_trajectory->at(i) - m_trajectory->at(i-1)).abs());
  }

  // overall length
  m_length = 0.0;
  std::for_each(m_distances.begin(), m_distances.end(), [&](double d){m_length += d;});
}

void Trajectory::removeDuplicatePoints()
{
  Point point_before = m_trajectory->front();
  for (std::vector<Point>::iterator it = m_trajectory->begin() + 1; it != m_trajectory->end(); ++it)
  {
    while(*it == point_before)
    {
      it = m_trajectory->erase(it);
    }
    point_before = *it;
  }
}


} //ns
