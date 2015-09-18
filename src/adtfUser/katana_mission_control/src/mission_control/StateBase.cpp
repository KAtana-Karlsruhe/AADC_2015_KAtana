// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-08
 *
 */
//----------------------------------------------------------------------

#include "mission_control/StateBase.h"

namespace katana
{

void StateBase::init(const std::function<void(u_int32_t)>& change_state)
{
  m_change_state = change_state;
}


} // ns
