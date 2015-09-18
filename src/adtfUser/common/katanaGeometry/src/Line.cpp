#include "Line.h"

namespace katana
{

Point Line::getPointOnLine(tFloat32 splitFactor)
{
  assert(splitFactor <= 1);
  assert(splitFactor >= 0);
  
  _position_type xValue = (1-splitFactor) * m_startPoint.x() + splitFactor * m_endPoint.x();
  _position_type yValue = (1-splitFactor) * m_startPoint.y() + splitFactor * m_endPoint.y();
  
  return Point(xValue,yValue);
}


}