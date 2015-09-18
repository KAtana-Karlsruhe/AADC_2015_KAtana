// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author
 * \date    2015-02-03
 *
 */
//----------------------------------------------------------------------

#ifndef _HELPER_CLASSES_TRAJECTORY_INTERPOLATOR_H
#define _HELPER_CLASSES_TRAJECTORY_INTERPOLATOR_H

#include <ExtendedPoint.h>
#include <vector>

namespace katana
{
typedef std::vector<ExtendedPoint> ExtendedTrajectory;

class TrajectoryInterpolator
{
public:

  //! Default constructor
  TrajectoryInterpolator();

  //! Destructor
  ~TrajectoryInterpolator();

  //! Apply a constant velocity to the whole trajectory
  static void applyConstantVelocity(ExtendedTrajectory& trajectory, double velocity);

  /*! Set two velocity values \a velocity_from at \a index_from and
   *  \a velocity_to at \a index_to and linearly interpolate
   *  the velocities in between those indices.
   *  Other indices are left unchanged.
   *  Requires index_from to be smaller than index_to.
   */
  static void applyLinearVelocityRamp(ExtendedTrajectory& trajectory,
                                      std::size_t index_from, std::size_t index_to,
                                      double velocity_from, double velocity_to);

  /*! Calculate the length of the \a trajectory when walking along each segment
   *  from \a index_from to \a index_to
   */
  static double length(const ExtendedTrajectory& trajectory, std::size_t index_from, std::size_t index_to);

  //! Write data contained in \a trajectory to data and gnuplot file
  static void toGnuplot(const ExtendedTrajectory& trajectory,
                        const std::string& filename_without_suffix);

private:

  /*! Perform linear interpolation between values \x and \y with
   *  a given \a ratio. A ratio within (0, 1) will result in a value
   *  within the interval (x, y).
   */
  static double interpolateLinear(double x, double y, double ratio);

  /*! Perform spline interpolation between values \x and \y with
   *  a given \a ratio. A ratio within (0, 1) will result in a value
   *  within the interval (x, y).
   *  The values \a a and \a b are used as support.
   *  The spline is created using Catmul-Rom polygon coefficients.
   */
  static double interpolateSpline(double a, double x, double y, double b, double ratio);

  /*! Perform spline interpolation between points \x and \y with
   *  a given \a ratio. A ratio within (0, 1) will result in a point
   *  in between (x, y).
   *  The points \a a and \a b are used as support.
   *  The spline is created using Catmul-Rom polygon coefficients.
   */
  static Point interpolateSpline(const Point& a, const Point& x, const Point& y, const Point& b, double ratio);

};

} //ns

#endif //_HELPER_CLASSES_TRAJECTORY_INTERPOLATOR_H
