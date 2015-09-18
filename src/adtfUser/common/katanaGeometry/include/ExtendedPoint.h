// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Philipp Hertweck
 *
 */
//----------------------------------------------------------------------

#ifndef KATANA_COMMON_EXTENDED_POINT_H
#define KATANA_COMMON_EXTENDED_POINT_H

#include <stdlib.h>
#include <cmath>
#include <utils/base/types.h>
#include "Point.h"
#include "Angle.h"

namespace katana
{

class ExtendedPoint : public katana::Point, public katana::Angle
{
public:

  ExtendedPoint()
    : m_angle(0),
      m_kappa(0)
  {}

  ExtendedPoint(_position_type x, _position_type y, _angle_type theta, double kappa)
  {
   setX(x);
   setY(y);
   m_angle = Angle(theta);
  }

  ~ExtendedPoint()
  {}

  //! Setter
  virtual void setKappa(double kappa)		{ m_kappa = kappa; }

  //! Write access via reference
  Angle& angle() { return m_angle; }
  double& kappa() { return m_kappa; }
  double& velocity() { return m_velocity; }

  //! Read access via reference
  const Angle& angle() const { return m_angle; }
  const double& kappa() const { return m_kappa; }
  const double& velocity() const { return m_velocity; }

  // todo: maybe remove these if they are not needed
  //const Angle& getAngle() const { return m_angle; }
  //const float& getKappa() const { return m_kappa; }

private:
  //! The orientation part in rad
  Angle	m_angle;

  /*! The curvature.
   *  Left turns are negative!
   *  Right turns are positive!
   *  This mimics the behaviour of Audi steering wheel implementation.
   */
  double m_kappa;

  //! The velocity to drive in [mm/10]
  double m_velocity;

};

} // ns

#endif //KATANA_COMMON_EXTENDED_POINT_H
