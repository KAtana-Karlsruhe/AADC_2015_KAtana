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

#ifndef _MISSION_CONTROL_CONFIG_H
#define _MISSION_CONTROL_CONFIG_H

#include "katanaCommon/katanaCommon.h"

namespace katana
{

enum DoubleParameter : u_int16_t
{
  CONF_STANDARD_VELOCITY=0,
  CONF_PID_TA=1,
  CONF_PID_KP=2,
  CONF_PID_KI=3,
  CONF_PID_KD=4,
  CONF_VELOCITY_AFTER_LANE_CHANGE=5,
  CONF_JUNCTION_MISSED_DISTANCE=6,
  CONF_MIN_ROADSIGN_JUNCTION_CHANGE=7, //< min area of sign needed to trigger junction state
  CONF_MIN_PARKING_SIGN_CHANGE=8,      //< min area of parking sign needed to trigger parking state
  CONF_WAIT_TIME_JUNCTION=9,
  CONF_DISTANCE_NEEDED_FOR_LANE_CHANGE=10,
  CONF_DISTANCE_NORMAL_SPEED_AFTER_JUNCTION=11,
  CONF_DISTANCE_NORMAL_SPEED=12,
  CONF_OBSTACLE_WAITING_DURATION=13,
  DOUBLE_PARAMETER_COUNT=14             //< keep this uptodate!
};

enum IntParameter : u_int16_t
{
  BLA=0,
  INT_PARAMETER_COUNT=1
};

class Config
{
public:



  static const char * const DOUBLE_IDENTIFIER[];
  static const char * const INT_IDENTIFIER[];


  //! Convenience pointer
  typedef std::shared_ptr<Config> Ptr;

  //! Constructor
  Config()
  {

  }

  //! Destructor
  virtual ~Config()     {}

  //!
  bool readFromFile(const std::string& file);

  //! ACCESS
  const double& getDouble(DoubleParameter d) const    { return m_doubles[d]; }
  int32_t getInt(IntParameter i) const                { return m_doubles[i]; }

private:

  static const constexpr char* XML_INT = "int";
  static const constexpr char* XML_DOUBLE = "double";
  static const constexpr char* XML_NAME = "name";
  static const constexpr char* XML_VALUE = "value";

  bool setDoubleValue(const char* name, const double& value);
  bool setIntValue(const char* name, const int32_t& value);

  //! integer
  int32_t m_ints[INT_PARAMETER_COUNT];

  //! double
  double m_doubles[DOUBLE_PARAMETER_COUNT];

};


} // ns

#endif //_MISSION_CONTROL_CONFIG_H
