// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-11-17
 *
 */
//----------------------------------------------------------------------

#include "Pose.h"

namespace katana
{

Pose Pose::transformToWorld(const Pose& p) const
{
  double sin_a = sin(getTheta());
  double cos_a = cos(getTheta());

  Pose result;

  // Rotation
  result.setX(p.getX()*cos_a - p.getY()*sin_a);
  result.setY(p.getX()*sin_a + p.getY()*cos_a);

  // Translation
  (Point)result.operator+=(*this);    // explicitly call point operator, which does not take care of angle and will neither in the future

  // Rotate object pose
  result.setTheta(p.getTheta() + getTheta());

  return result;
}

Point Pose::transformToWorld(const Point& p) const
{
  Pose tmp(p.getX(), p.getY(), 0);
  return transformToWorld(tmp);
}

void Pose::transformToWorld(vector<Pose>& input) const
{
  double sin_a = sin(getTheta());
  double cos_a = cos(getTheta());

  for (Pose& p : input)
  {
    // Rotation
    // use round to minimize error when converting to .1mm integer
    _position_type x_rot = p.getX()*cos_a - p.getY()*sin_a;
    _position_type y_rot = p.getX()*sin_a + p.getY()*cos_a;
    p.setX(x_rot);
    p.setY(y_rot);

    // Translation
    (Point)p.operator+=(*this);
    
    // Rotate object pose
    p.setTheta(p.getTheta() + getTheta());
  }
}

void Pose::transformToWorld(std::map<ObstacleSource, Pose>& input) const
{
  double sin_a = sin(getTheta());
  double cos_a = cos(getTheta());

  for(std::map<ObstacleSource, Pose>::iterator it = input.begin(); it != input.end(); ++it)
  {
    Pose& p = it->second;
    // Rotation
    // use round to minimize error when converting to .1mm integer
    _position_type x_rot = p.getX()*cos_a - p.getY()*sin_a;
    _position_type y_rot = p.getX()*sin_a + p.getY()*cos_a;
    p.setX(x_rot);
    p.setY(y_rot);

    // Translation
    (Point)p.operator+=(*this);

    // Rotate object pose
    p.setTheta(p.getTheta() + getTheta());
  }
}


Pose Pose::transformToVehicle(const Pose &p) const
{
  double sin_a = sin(getTheta());
  double cos_a = cos(getTheta());

  Pose result = p;

  // Translation
  (Point)result.operator-=(*this);    // explicitly call point operator, which does not take care of angle and will neither in the future

  // Rotation
  // use round to minimize error when converting to .1mm integer
  _position_type x_rot = result.getX()*cos_a + result.getY()*sin_a;
  _position_type y_rot = -result.getX()*sin_a + result.getY()*cos_a;
  result.setX(x_rot);
  result.setY(y_rot);

  // Rotate object pose
  result.setTheta(p.getTheta() - getTheta());

  return result;
}

Point Pose::transformToVehicle(const Point& p) const
{
  Pose tmp(p.getX(), p.getY(), 0);
  return transformToVehicle(tmp);
}

void Pose::transformToVehicle(std::vector<Pose>& input) const
{
  double sin_a = sin(getTheta());
  double cos_a = cos(getTheta());

  for (Pose& p : input)
  {
    // Translation
    (Point)p.operator-=(*this);    // explicitly call point operator, which does not take care of angle and will neither in the future

    // Rotation
    // use round to minimize error when converting to .1mm integer
    _position_type x_rot = p.getX()*cos_a + p.getY()*sin_a;
    _position_type y_rot = -p.getX()*sin_a + p.getY()*cos_a;
    p.setX(x_rot);
    p.setY(y_rot);

    // Rotate object pose
    p.setTheta(p.getTheta() - getTheta());
  }
}

} //ns
