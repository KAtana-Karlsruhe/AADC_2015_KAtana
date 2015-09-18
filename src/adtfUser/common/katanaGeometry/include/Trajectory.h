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

#ifndef _HELPER_CLASSES_TRAJECTORY_H
#define _HELPER_CLASSES_TRAJECTORY_H

#include "katanaCommon/katanaCommon.h"
#include "Pose.h"
#include "ExtendedPoint.h"

#include <vector>

namespace katana
{

class Trajectory
{
public:
  //! Convenience Pointer
  typedef std::shared_ptr<Trajectory> Ptr;
  //! Convenience const Pointer
  typedef std::shared_ptr<const Trajectory> ConstPtr;

  //! Pointer for extended points
  typedef std::vector<Point> PointContainer;
  typedef std::shared_ptr<PointContainer> PointContainerPtr;
  typedef std::shared_ptr<const PointContainer> PointContainerConstPtr;

  //! Constructor
  Trajectory();
  //! Copy Contructor (we need a deep copy, the PointContainer has to be duplicated!)
  Trajectory(const Trajectory& t);
  //! Assignment
  Trajectory& operator =(const Trajectory& t);

  //! Destructor
  virtual ~Trajectory();

  bool empty() const                                { return m_trajectory->empty(); }

  //! Read access
  PointContainerConstPtr getTrajectoryPtr() const   { return m_trajectory; }
  const PointContainer& getTrajectory() const       { return *m_trajectory; }

  //! Change all points to world coordinate system
  void transformToWorld(const Pose& base);

  //! Append another trajectory
  void append(const Trajectory& t);
  void append(const Point& p);
  void append(const PointContainer& points);
  
  //! Insert an other trajectory ahead
  void insertAhead(const Trajectory& t);
  
  //! Remove duplicate points in trajectory if they are next to each other
  void removeDuplicatePoints();

  //!
  void createFromSubTrajectories(const std::vector<Trajectory::ConstPtr>& trajectories);

  //! Adjust points to match given end point -> last point will be target, first point will keep its position
  void adjustTrajectory(const Point& target);

private:
  //!
  void updateDistances();

  //! Driving points
  PointContainerPtr m_trajectory;

  //! Distances between adjacent points
  std::vector<double> m_distances;
  //! Overall length of driving polygon
  double m_length;
};


} //ns

#endif //_HELPER_CLASSES_TRAJECTORY_H
