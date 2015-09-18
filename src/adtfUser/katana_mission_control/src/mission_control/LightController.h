// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Raphael Frisch <frisch@fzi.de>
 * \date    2014-11-15
 *
 */
//----------------------------------------------------------------------

#ifndef _LIGHT_CONTROLLER_H_
#define _LIGHT_CONTROLLER_H_

#include <vector>

#include "katanaCommon/katanaCommon.h"

namespace katana
{

enum LightName{
  HEAD_LIGHT,
  BREAK_LIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  REVERSE_LIGHT,
  WARNING_LIGHTS,
  NUMBER_OF_LIGHTS = 6
};
//! Array storing the state of the lights
//! Order: head_light, break_light, turn_left, turn_right, reverse_light
//! Always turn head_light on!!!
//! 0 = off, 1 = on, 2 = don't care
typedef std::array<u_int8_t, NUMBER_OF_LIGHTS> LightState;

//! Light transmit function
typedef std::function<void(const katana::LightState lightArray)> LightStateFunc;


class LightController
{
public:
  //! Convenience pointer
  typedef std::shared_ptr<LightController> Ptr;

  //! Constructor
  LightController() = delete;
  LightController(LightStateFunc func);

  //! Destructor
  virtual ~LightController()    {}

  void setLight(LightName name, bool set);
  void send() const { m_output_light(m_state); }

  void resetLights();

private:
  //! Light Array
  LightState m_state;

  LightStateFunc m_output_light;

};

} // ns

#endif // _LIGHT_CONTROLLER_H
