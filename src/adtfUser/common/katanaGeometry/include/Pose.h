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

#ifndef KATANA_COMMON_POSE_H
#define KATANA_COMMON_POSE_H

#include <memory>
#include "Angle.h"
#include "Point.h"

#include <vector>
#include <map>

using namespace std;

namespace katana
{

class Pose : public Point, public Angle
{
public:
  Pose()
    : Point()
    , Angle()
  {
  }
  Pose(_position_type x, _position_type y, _angle_type theta)
    : Point(x, y)
    , Angle(theta)
  {
  }

  virtual ~Pose()   {}

  //! Interprets *this as vehicle pose and transforms given pose from vehicle to world coordinate system
  Pose transformToWorld(const Pose& p) const;

  //!
  Point transformToWorld(const Point& p) const;
  void transformToWorld(std::vector<Pose>& input) const;
  void transformToWorld(std::map<ObstacleSource, Pose>& input) const;


  //! Interprets *this as vehicle pose and transforms given pose from world to vehicle coordinate system
  Pose transformToVehicle(const Pose& p) const;
  void transformToVehicle(std::vector<Pose>& input) const;

  //!
  Point transformToVehicle(const Point& p) const;


  virtual bool operator ==(const Pose& rhs) const             { return equals(rhs); }

  virtual bool equals(const Pose& p) const                    { return p.getX() == x() && p.getY() == y() && p.getTheta() == getTheta(); }

  virtual void printToConsole()	const override                { std::cout << "Pose (" << x() << "," << y() <<"," << getTheta() << ")" << std::endl; }

protected:


private:

};

}
#endif //KATANA_COMMON_POSE_H
