// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-11-23
 *
 */
//----------------------------------------------------------------------

#include "Angle.h"

namespace katana
{

void Angle::setTheta(_angle_type theta)
{
  while (std::abs(theta) > M_PI)
    theta = (theta >= 0) ? theta - 2*M_PI : theta + 2*M_PI;

  m_theta = theta;
}

Angle Angle::getReverseDirection() const
{
  return Angle(m_theta + M_PI);
}

}   //ns
