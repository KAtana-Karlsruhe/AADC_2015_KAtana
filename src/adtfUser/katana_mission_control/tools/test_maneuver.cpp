// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-11-30
 *
 */
//----------------------------------------------------------------------

#include <iostream>

#include "mission_control/maneuver.h"
#include "RoadPatch.h"

/*************** JUST FOR DEBUGGING *********************/

int main(int argc, char* argv[])
{
  //katana::Maneuver m;
  //m.readManeuver("/home/christoph/Workspace/aadc/src/adtfUser/katana_mission_control/etc/route.xml");

  std::shared_ptr<katana::RoadPatch::PatchArray> tmp = katana::RoadPatch::loadPatches("/home/christoph/Workspace/aadc/src/adtfUser/common/patches/patches.xml");

  std::cout <<(u_int32_t)((*tmp)[1])->getPatchType() <<std::endl;



  /*if (rp.good())
  {
    std::cout <<"ID: " <<(u_int32_t)rp.getPatchType() <<std::endl;

    for (const katana::Point& i : *rp.getMedianStrip())
      std::cout <<"X: " <<i.getX() <<" Y: " <<i.getY() <<std::endl;
  }*/

  return 0;
}
