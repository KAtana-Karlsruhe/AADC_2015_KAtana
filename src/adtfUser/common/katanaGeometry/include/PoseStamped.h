// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-01-13
 *
 */
//----------------------------------------------------------------------

#ifndef KATANA_COMMON_POSE_STAMPED_H
#define KATANA_COMMON_POSE_STAMPED_H

#include "Pose.h"

using namespace std;

namespace katana
{

class PoseStamped : public Pose
{
public:
  typedef int64_t _stamp_type;

  PoseStamped()
    : Pose()
    , m_stamp(0)
  {

  }
  PoseStamped(_position_type x, _position_type y, _angle_type theta, _stamp_type stamp = 0)
    : Pose(x, y, theta)
    , m_stamp(stamp)
  {
  }
  PoseStamped(const Pose& p, _stamp_type stamp)
    : Pose(p)
    , m_stamp(stamp)
  {

  }

  _stamp_type getStamp() const        { return m_stamp; }
  void setStamp(_stamp_type stamp)    { m_stamp = stamp; }

protected:
  _stamp_type m_stamp;

private:

};

}
#endif //KATANA_COMMON_POSE_STAMPED_H
