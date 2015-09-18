// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-07
 *
 */
//----------------------------------------------------------------------

#include <iostream>

#include "mission_control/PositionController.h"
#include "RoadPatch.h"

/*************** JUST FOR DEBUGGING *********************/

using namespace katana;

void dummy(u_int8_t steering)
{

}

void dummy2(u_int8_t speed)
{

}

int main(int argc, char* argv[])
{
  PositionController pc(&dummy, &dummy2, katana::TransmitStatusFunc());

  std::this_thread::sleep_for(std::chrono::seconds(5));

  return 0;
}
