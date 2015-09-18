// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-22
 *
 */
//----------------------------------------------------------------------

#include "Config.h"

#include <tinyxml2.h>

#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
#include <iostream>
#endif

using namespace tinyxml2;

namespace katana
{

const char * const Config::DOUBLE_IDENTIFIER[] =
{
  "standard_velocity",
  "pid_ta",
  "pid_kp",
  "pid_ki",
  "pid_kd",
  "velocity_after_lane_change",
  "junction_missed_distance",
  "min_roadsign_junction_change",
  "min_parking_sign_change",
  "wait_time_junction",
  "distance_neede_for_lane_change",
  "distance_normal_speed_after_junction",
  "distance_normal_speed",
  "obstacle_waiting_duration"
};

const char * const Config::INT_IDENTIFIER[] =
{
  "bla"
};

bool Config::readFromFile(const std::string &file)
{
  XMLDocument doc;
  doc.LoadFile(file.c_str());

  if (doc.Error())
    return false;

  // read doubles
  double double_value;

  for (XMLElement* double_element = doc.FirstChildElement(XML_DOUBLE);
       double_element != nullptr;
       double_element = double_element->NextSiblingElement(XML_DOUBLE))
  {
    const char* name = double_element->Attribute(XML_NAME);
    double_value = double_element->DoubleAttribute(XML_VALUE);

    if (!setDoubleValue(name, double_value))
      return false;
  }


  // read ints
  int32_t int_value;

  for (XMLElement* int_element = doc.FirstChildElement(XML_INT);
       int_element != nullptr;
       int_element = int_element->NextSiblingElement(XML_INT))
  {
    const char* name = int_element->Attribute(XML_NAME);
    int_value = int_element->IntAttribute(XML_VALUE);

    if (!setIntValue(name, int_value))
      return false;
  }

#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
  std::cout <<"**** Read Config: ****" <<std::endl;
  for (u_int16_t i = 0; i < DOUBLE_PARAMETER_COUNT; i++)
  {
    std::cout <<" - " <<DOUBLE_IDENTIFIER[i] <<"\t\t" <<m_doubles[i] <<std::endl;
  }
  for (u_int16_t i = 0; i < INT_PARAMETER_COUNT; i++)
  {
    std::cout <<" - " <<INT_IDENTIFIER[i] <<"\t\t" <<m_ints[i] <<std::endl;
  }
#endif

  return true;
}

bool Config::setDoubleValue(const char *name, const double &value)
{
  for (u_int16_t i = 0; i < DOUBLE_PARAMETER_COUNT; i++)
  {
    if (strcmp(name, DOUBLE_IDENTIFIER[i]) == 0)
    {
      m_doubles[i] = value;
      return true;
    }
  }
#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
  std::cout <<"No match for property " <<name <<std::endl;
#endif

  return false;
}

bool Config::setIntValue(const char *name, const int32_t& value)
{
  for (u_int16_t i = 0; i < DOUBLE_PARAMETER_COUNT; i++)
  {
    if (strcmp(name, INT_IDENTIFIER[i]) == 0)
    {
      m_ints[i] = value;
      return true;
    }
  }

#ifdef KATANA_MC_IMPORTANT_DEBUG_MSG
  std::cout <<"No match for property " <<name <<std::endl;
#endif

  return false;
}

} // ns
