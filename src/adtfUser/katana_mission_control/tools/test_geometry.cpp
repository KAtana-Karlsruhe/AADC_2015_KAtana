// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-05
 *
 */
//----------------------------------------------------------------------

#include <iostream>

#include "Pose.h"
#include "mission_control/mission_control.h"

/*************** JUST FOR DEBUGGING *********************/

using namespace std;
using namespace katana;

void dummy(u_int8_t steering)
{

}

void dummy2(u_int8_t speed)
{

}

int main(int argc, char* argv[])
{
  /*
  // vehicle pose
  Pose v(1, 1, M_PI/2);
  // object pose in vehicle coordinates
  Pose p(1, 2, 0.5);

  cout <<"Ego:  X " <<v.x() <<" Y " <<v.y() <<" T " <<v.getTheta() <<endl;

  while (true)
  {
    Pose w = v.transformToWorld(p);
    cout <<"World:  X " <<w.x() <<" Y " <<w.y() <<" T " <<w.getTheta() <<endl;

    cin >> p.x(); cout <<endl;
    cin >> p.y(); cout <<endl;
    _angle_type theta;
    cin >> theta; cout <<endl;
    p.setTheta(theta);
  }
  */



  MissionControl mc;
  /*mc.initialize("/home/christoph/Workspace/aadc/src/adtfUser/katana_mission_control/etc/aadc_route.xml",
                "/home/christoph/Workspace/aadc/src/adtfUser/common/patches/patches.xml",
                std::function<void(u_int8_t)>(dummy),
                std::function<void(u_int8_t)>(dummy2),
                katana::TransmitPatchesFunc(),
                std::function<void(u_int8_t, int32_t)>()
                );*/


  //mc.getStateObject()->poseChanged(Pose(1, 2, 0.3));

  return 0;
}
