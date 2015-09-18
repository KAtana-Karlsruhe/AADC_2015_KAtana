// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-07
 *
 */
//----------------------------------------------------------------------

#include "Obstacle.h"

using namespace oadrive::core;

namespace katana
{

void Obstacle::calculateQuadrangle(oadrive::core::Pose2d p, BoundingBox bounding_box)
{
  m_quadrangle[0] = Position2d(-bounding_box.first/2, bounding_box.second/2);
  m_quadrangle[1] = Position2d(-bounding_box.first/2, -bounding_box.second/2);
  m_quadrangle[2] = Position2d(bounding_box.first/2, -bounding_box.second/2);
  m_quadrangle[3] = Position2d(bounding_box.first/2, bounding_box.second/2);

  for(size_t i = 0; i < 4; i++)
  {
    m_quadrangle[i] = (p * m_quadrangle[i]).eval();
  }
}


} //ns
