// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-10
 *
 */
//----------------------------------------------------------------------

#ifndef _KATANA_WORLD_POSEWITHTIME_H
#define _KATANA_WORLD_POSEWITHTIME_H

#include "katanaCommon/katanaCommon.h"

#include "Obstacle.h"
#include <oadrive_core/Pose.h>


using namespace oadrive::core;

namespace katana
{

typedef u_int64_t _time_type;

struct PoseWithTime
{
  Pose2d pose;
  _time_type time;
};


} // ns

#endif //_KATANA_WORLD_POSEWITHTIME_H
