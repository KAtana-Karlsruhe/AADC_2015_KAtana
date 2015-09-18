#include "LightController.h"

#include <cmath>
#include <oadrive_core/Interpolator.h>


namespace katana
{

LightController::LightController(LightStateFunc func)
  : m_output_light(func)
{
  for ( size_t i =0; i< NUMBER_OF_LIGHTS;++i)
    m_state[i] = 0;
}

void LightController::setLight(LightName name, bool set)
{
  m_state[name] = (u_int8_t)set;

  LightState send;
  for ( size_t i =0; i< NUMBER_OF_LIGHTS;++i)
    send[i] = (i == name) ? (u_int8_t)set : 2;

  m_output_light(send);
}

void LightController::resetLights()
{
  LightState send;
  for ( size_t i =0; i< NUMBER_OF_LIGHTS;++i)
    send[i] = 0;
  m_output_light(send);
}

} // ns
