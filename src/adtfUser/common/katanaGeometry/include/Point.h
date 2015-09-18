// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \author  Philipp Hertweck <hertweck@fzi.de>
 * \date    2014-11-17
 *
 */
//----------------------------------------------------------------------

#ifndef KATANA_COMMON_POINT_H
#define KATANA_COMMON_POINT_H

#include <stdlib.h>
#include <utils/base/types.h>
#include <cmath>
#include <memory>
#include <iostream>
#include "Angle.h"
#include "katanaCommon/katanaCommon.h"

namespace katana
{

enum pointDirection
{
  onLine = 0,
  left = -1,
  right = 1
};

class Point
{
public:
  typedef std::shared_ptr<Point> Ptr;

  Point()
    : m_x(0),
      m_y(0)
  {}

  Point(_position_type x, _position_type y);

  ~Point()
  {}

  //! Getter
  _position_type getX() const { return m_x; }
  _position_type getY() const { return m_y; }

  //! Setter
  void setX(_position_type x) { m_x = x; }
  void setY(_position_type y) { m_y = y; }

  //! Access via reference
  _position_type& x() { return m_x; }
  _position_type& y() { return m_y; }

  const _position_type& x() const { return m_x; }
  const _position_type& y() const { return m_y; }

  _position_type distanceToPoint(const Point& p) const      { return Point(p - *this).abs();}
  static pointDirection getPointDirection(const Point& p1, const Point& p2, const Point& p3);

  Angle getAngleBetweenPoints(const Point& otherPoint) const;

  //! angle in radians between the positive x-axis of a plane and the this point given by the coordinates (x, y) on it
  Angle getAngle() const;

  virtual double abs() const;


  virtual Point operator + (const Point& rhs) const    { return Point(m_x + rhs.getX(), m_y + rhs.getY()); }
  virtual Point operator - (const Point& rhs) const    { return Point(m_x - rhs.getX(), m_y - rhs.getY()); }
  virtual Point operator * (double factor) const       { return Point(m_x * factor, m_y * factor); }
  virtual Point& operator += (const Point& rhs)        { m_x += rhs.getX(); m_y += rhs.getY(); return *this; }
  virtual Point& operator -= (const Point& rhs)        { m_x -= rhs.getX(); m_y -= rhs.getY(); return *this; }
  virtual Point& operator *= (double factor)           { m_x *= factor; m_y *= factor; return *this; }

  virtual bool operator == (const Point& rhs) const    { return equals(rhs); }

  virtual bool equals(const Point& p) const   	       { return p.getX() == m_x && p.getY() == m_y; }

  // For debugging
  virtual void printToConsole()	const                  { std::cout << "Point (" << m_x << "," << m_y << ")" << std::endl; }

private:

  //! the point's x coordinate in [mm/10]
  _position_type m_x;
  //! the point's y coordinate in [mm/10]
  _position_type m_y;

};

} // ns

#endif //KATANA_COMMON_POINT_H
