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

#ifndef KATANA_COMMON_ANGLE_H
#define KATANA_COMMON_ANGLE_H

#include <stdlib.h>
#include <cmath>

namespace katana
{

typedef float _angle_type;

class Angle
{
public:
  Angle()
    : m_theta(0)
  {}

  Angle(_angle_type theta)
    : m_theta(theta)
  {}

  ~Angle()
  {}

  //! Getter
  const _angle_type& getTheta() const    { return m_theta; }

  //! Setter
  void setTheta(_angle_type theta);

  //! Access by referende
  const _angle_type& theta() const  { return m_theta; }
  _angle_type& theta()              { return m_theta; }

  //!
  Angle& operator = (_angle_type rhs)
  {
    setTheta(rhs);
    return *this;
  }

  //!
  _angle_type operator () () const      { return m_theta; }

  //! Invert angle
  Angle getReverseDirection() const;

protected:


private:
  //! The angle in radians
  _angle_type m_theta;
};

} // ns

#endif //KATANA_COMMON_ANGLE_H
