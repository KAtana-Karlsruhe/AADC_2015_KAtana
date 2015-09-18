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

#include "TrajectoryInterpolator.h"

#include "katanaCommon/katanaCommon.h"
#include <fstream>

namespace katana
{

void TrajectoryInterpolator::applyConstantVelocity(ExtendedTrajectory& trajectory, double velocity)
{
  for (std::size_t i=0; i<trajectory.size(); ++i)
  {
    trajectory[i].velocity() = velocity;
  }
}

void TrajectoryInterpolator::applyLinearVelocityRamp(ExtendedTrajectory& trajectory,
                                                     size_t index_from, size_t index_to,
                                                     double velocity_from, double velocity_to)
{
  assert((index_from < trajectory.size()) && "index_from outside trajectory scope!");
  assert((index_to < trajectory.size()) && "index_to outside trajectory scope!");
  assert((index_from < index_to) && "index_from must be smaller than index_to!");

  trajectory[index_from].velocity() = velocity_from;
  trajectory[index_to].velocity()   = velocity_to;

  const double whole_length = length(trajectory, index_from, index_to);
  double length_so_far = length(trajectory, index_from, index_from + 1);
  double ratio;
  for (std::size_t i=index_from+1; i<index_to; ++i)
  {
    ratio = length_so_far / whole_length;
    trajectory[i].velocity() = interpolateLinear(velocity_from, velocity_to, ratio);
    length_so_far += length(trajectory, i, i+1);
  }
}

double TrajectoryInterpolator::interpolateLinear(double x, double y, double ratio)
{
  return (x * (1.0 - ratio) + y * ratio);
}

double TrajectoryInterpolator::interpolateSpline(double a, double x, double y, double b, double ratio)
{
  // coefficients:ﬁ
  const double a0 = -0.5*a + 1.5*x - 1.5*y + 0.5*b;
  const double a1 = a - 1.5*x + 2*y - 0.5*b;
  const double a2 = -0.5*a + 0.5*y;
  const double ratio_square = ratio * ratio;
  return (a0 * ratio * ratio_square + a1 * ratio_square + a2 * ratio + x);
}


Point TrajectoryInterpolator::interpolateSpline(const Point& a, const Point& x, const Point& y, const Point& b, double ratio)
{
  Point interpolated;
  interpolated.x() = interpolateSpline(a.x(), x.x(), y.x(), b.x(), ratio);
  interpolated.y() = interpolateSpline(a.y(), x.y(), y.y(), b.y(), ratio);
  return interpolated;
}



double TrajectoryInterpolator::length(const ExtendedTrajectory& trajectory,
                                      std::size_t index_from, std::size_t index_to)
{
  double length = 0.;
  for (std::size_t i=index_from; i<index_to; ++i)
  {
    length += trajectory[i].distanceToPoint(trajectory[i+1]);
  }
  return length;
}

void TrajectoryInterpolator::toGnuplot(const ExtendedTrajectory& trajectory,
                                       const std::string& filename_without_suffix)
{
  const std::string gnuplot_filename = filename_without_suffix + ".gpl";
  const std::string data_filename = filename_without_suffix + ".gpldata";

  // -- data formatting --
  std::ofstream data_file(data_filename.c_str(), std::ios::out);
  data_file << "# Trajectory data set containing "<< trajectory.size() << " sets of position, orientation, curvature and velocity information." << std::endl;

  for (std::size_t i=0; i<trajectory.size(); ++i)
  {
    data_file << trajectory[i].x() << " "
              << trajectory[i].y() << " "
              << trajectory[i].angle().getTheta() << " "
              << trajectory[i].kappa() << " "
              << trajectory[i].velocity() << std::endl;
  }

  data_file << std::endl;
  data_file.close();

  // -- the plot itself --

  std::ofstream gnuplot_file(gnuplot_filename.c_str(), std::ios::out);
  gnuplot_file << "# Gnuplot file. Draw with gnuplot -p <filename>." << std::endl;
  gnuplot_file << "set mapping cartesian" << std::endl;
  gnuplot_file << "set mouse" << std::endl;
  gnuplot_file << "splot '"<< data_filename << "' using 1:2:5 w lp" << std::endl;
  gnuplot_file.close();
  std::cout << "TrajectoryInterpolator::toGnuplot(): GnuPlot file written to " << gnuplot_filename << std::endl;
}

} //ns
