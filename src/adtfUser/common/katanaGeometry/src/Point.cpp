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

#include "Point.h"

namespace katana
{


Point::Point(_position_type x, _position_type y)
  : m_x(x),
    m_y(y)
{
}

pointDirection Point::getPointDirection(const Point& p1, const Point& p2, const Point& p3)
{
  double cartesian_product = (p2.getX()-p1.getX())*(p3.getY()-p1.getY()) - (p3.getX()-p1.getX())*(p2.getY()-p1.getY());

  /*
  std::cout << "cartesion_product: " << cartesian_product << std::endl;
  p1.printToConsole();
  p2.printToConsole();
  p3.printToConsole();
  */

  if(std::abs(cartesian_product) < 1) {
   return onLine;
  } else if(cartesian_product > 0) {
    return left;
  } else {
    return right;
  }
}

double Point::abs() const
{
  return sqrt((double)getX()*getX() + (double)getY()*getY());
}

Angle Point::getAngle() const
{
  return Angle(atan2(m_y, m_x));
}

}   //ns
