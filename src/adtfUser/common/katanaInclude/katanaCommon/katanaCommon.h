// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2014-12-04
 *
 */
//----------------------------------------------------------------------

#ifndef KATANA_COMMON_INCLUDE
#define KATANA_COMMON_INCLUDE

// Assertion switch, define BEFORE #include <assert.h>
//#define NDEBUG
#undef NDEBUG

#include <memory>
#include <assert.h>
#include <functional>
//#include <cstdint>
#include <stdint.h>
#include <math.h>

/*********** MISSION_CONTROL DEBUG **************/
#define KATANA_MC_IMPORTANT_DEBUG_MSG
/*< ONLY STATE CHANGES OF MISSION CONTROL - QUITE IMPORTANT >*/
#define KATANA_MC_STATE_DEBUG

//#define KATANA_MC_GENERAL_DEBUG
//#define KATANA_MC_WORLD_DEBUG
//#define KATANA_WORLD_DEBUG_PLOT
//#define KATANA_MC_WORLD_OBSTACLE_DEBUG
//#define KATANA_MC_WORLD_SIGNS_DEBUG
//#define KATANA_MC_POSITION_CONTROLLER_DEBUG_PLOT
//#define KATANA_MC_POSITION_CONTROLLER_DEBUG
//#define KATANA_MC_LATERAL_CONTROLLER_DEBUG
//#define KATANA_MC_MOTION_PLANNER_DEBUG
//#define KATANA_MC_FILTER_DEBUG
//#define KATANA_MC_FILTER_TRANSMIT_DEBUG
//#define KATANA_MC_STATE_DEBUG
//#define KATANA_MC_VELOCITY_DEBUG
//#define KATANA_MC_SHOW_VEHICLE_POSE_INPUT
//#define KATANA_MC_JURY_DEBUG
//#define KATANA_MC_MANEUVER_DEBUG
//#define KATANA_MC_PATCH_TRANSMIT_DEBUG
#define MC_ENABLE_SVG_MAP
//#define MC_ENABLE_SVG_MAP_COUNTER
//#define KATANA_MC_JUNCTION_LOGIC_DEBUG
#define KATANA_REVERSE_DRIVING_DEBUG
//#define KATANA_MC_COLLISION_HANDLER
//#define KATANA_MC_PARKING_ASSISTANT_DEBUG
#define KATANA_MC_PULL_OUT_DEBUG
//#define KATANA_MC_LANETRACKER_JOBS_DEBUG
#define KATANA_MC_DRIVING_CHANGE_DEBUG
//#define KATANA_MC_CONFIG_DEBUG
// Enable plausibility checks in mission control for patches
//#define KATANA_MISSION_CONTROL_ENABLE_PATCH_PLAUSIBILITY_CHECKS

/********* ODOMETRY_SIMULATOR DEBUG *************/
//#define KATANA_ODOMETRY_SIMULATOR_DEBUG

/************** LANETRACKER_DEBUG ***************/
//#define KATANA_LT_DEBUG
//#define KATANA_LT_RECEIVE_PATCHES
//#define KATANA_LT_TRANSMIT_PATCHES
//#define KATANA_LT_WRITE_INPUT
//#define KATANA_LT_NEXT_RUN
//#define KATANA_LT_MEASURE_PERFORMANCE
//#define KATANA_LT_WRITE_PREPROCESSED

/********* ODOMETRY DEBUG *********/
//#define KATANA_ODOMETRY_DEBUG
//#define KATANA_ODOMETRY_DEBUB_PLOT
//#define KATANA_ODOMETRY_SIMULATOR_DEBUG

/********* IR/US OBSTACLES DEBUG *******/
//#define KATANA_IR_US_OBSTACLES_DEBUG

/********* KATANA ROAD SIGN DEBUG ******/
//#define KATANA_ROAD_SIGN_DEBUG

/*********** PICTURE REFUSAL ****************/
//#define KATANA_PICTURE_REFUSAL_DEBUG

/*********** TEST CASES ******************************/
// Leave this for executing test cases.
#define KATANA_TESTCASES_HELPER_METHODS

namespace katana {

// Coordinate system scaling
const double COORDINATE_SCALE_FACTOR_TO_M = 0.0001;   //scale from 0.1mm to m
const double COORDINATE_SCALE_FACTOR_FROM_M = 10000.0;   //scale from m to 0.1mm


// time in 10^-6 seconds
typedef int64_t _stamp_type;
typedef int64_t _clock_time;

// positioning
typedef double _position_type;
typedef float  _angle_type;

/* WORLD CONSTANTS */
//const _position_type DISTANCE_BETWEEN_POINTS = 500;
const _position_type DISTANCE_TO_DELETE_PATCHES = 4.0;
// World plausibility checks
//const int32_t MAX_DISTANCE_BETWEEN_PATCHES = 5000; // 50 cm
//const float MAX_ANGLE_BETWEEND_END_AND_START_POSE = M_PI/4; // 45 degrees
const double MAX_X_DIFF_SAME_PATCHES = 0.1;  // 10 cm; max allowed difference in x coordinates for patch with same id
const double MAX_Y_DIFF_SAME_PATCHES = 0.1;  // 10 cm;  // 10 cm; max allowed difference in y coordinates for patch with same id
//const float MAX_ANGLE_DIFF_SAME_PATCHES = M_PI/9; // 20 degrees;  // 10 cm; max allowed difference in angle for patch with same id

const int64_t TIME_TO_LIVE_OBSTACLES = 500000; // * 10^-6 seconds


/* MISSION CONTROL CONSTANTS */
const _position_type DRIVING_DISTANCE_TO_DRIVING_STRIP_UPDATE = 0.15;      //< 15cm
const u_int32_t MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER = 1;
const u_int32_t DEFAULT_DRIVING_STRIP = 0;
const u_int8_t STOP_LINE_WAITING_TIME = 15; // 3 s

/* POSITION CONTROLLER */
const double REACHED_ZONE_DISTANCE = 0.03;    //< 0.3cm

/* OBSTACLES CONSTATS */
static constexpr const u_int32_t    BOUNDING_X_STANDARD = 100;     // mm/10
static constexpr const u_int32_t    BOUNDING_Y_STANDARD = 500;     //mm/10

/* ROAD SIGN CONSTANTS */
//const float MIN_ROADSIGN_SIZE = 700; // area is in pixel*pixel
//const float MIN_ROADSIGN_JUNCTION_CHANGE = 800; // when to change to junction state
const int ROAD_SIGN_SKIP = 3; // Skip frames with (#frame % ROAD_SIGN_SKIP == 0)
const int IS_UPSIDE_DOWN_MARGIN = 2; // top-left corner has to be this much pixel above bottom left corner
//const float MIN_ROADSIGN_ASPECT = 0.7f;
//const float MAX_ROADSIGN_ASPECT = 1.05f;
#define KATANA_MD_ENABLE_ASPECT_FILTERING

//! Actions from AADC-Route file
enum class Action : int16_t {LEFT=0,
                             STRAIGHT=1,
                             RIGHT=2,
                             PARALLEL_PARKING=3,
                             CROSS_PARKING=4,
                             PULL_OUT_LEFT=5,
                             PULL_OUT_RIGHT=6,
                             UNKNOWN=7,
                             First=LEFT,
                             Last=UNKNOWN
                        };

enum class PatchType : int32_t {
        STRAIGHT=0,
        SMALL_L_CURVE=1,
        SMALL_R_CURVE=2,
        JUNCTION=3,
        CROSS_PARKING_REVERSE=4,
        CROSS_PARKING_FORWARD=5,
        PARALLEL_PARKING_REVERSE=6,
        PARALLEL_PARKING_FORWARD=7,
        PARALLEL_PARKING_FORWARD_2=8,
        PARALLEL_PARKING_PULL_OUT_REVERSE=9,
        PARALLEL_PARKING_PULL_OUT_FORWARD=10,
        PATCH_COUNT=11  //< keep this up-to-date!
};

//! Main states of mission control (provisional!)
enum class MainState : u_int8_t {
  ERROR=0,            // we have a serious problem, system down, no chance to continue at this point
  RECOVERING=1,       // something is wrong, try to recover
  WAITING=2,          // before Jury start, after Jury stop, DO NOT MOVE, but environment recognition is up and running
  DRIVING=3,          // standard driving (until junction, object, traffic sign, etc.)
  STATE_COUNT=4
};

//! Driving states
enum class DrivingState : u_int8_t {
  DRIVE_NORMAL=0,
  DRIVE_INITIALIZE=1,
  DRIVE_PARKING=2,
  DRIVE_JUNCTION=3,
  DRIVE_PULLOUT=4,
  DRIVE_FINISHED=5,
  DRIVE_STATE_COUNT=6
};

//! Traffic signs
enum class TrafficSign : u_int8_t {
  // Kreuzung mit Vorfahrt von rechts
  JUNCTION_PRIORITY_FROM_RIGHT = 0,
  // Vorfahrt gewähren
  JUNCTION_GIVE_WAY = 1,
  // Halt. Vorfahrt gewähren
  JUNCTION_STOP_GIVE_WAY = 2,
  // Vorfahrt
  JUNCTION_PRIORITY = 3,
  // Parken
  PARKING_AHEAD = 4,
  // Vorgeschriebene Fahrtrichtung
  PRESCRIBED_DIRECTION = 5,
  UNKNOWN = 6,
  SIGN_COUNT = 7
};

//! States relevant for perception
enum PerceptionState : u_int8_t {
  NORMAL = 0,                       //< normal driving
  RECOVERING = 1,                   //< search for everything, mission control has lost street
  INITIALIZE = 2,                   //< mission control thinks car is on road, sends guess to vision
  DO_NOTHING = 3,                   //< mission control does not need vision (e.g. reversing out of parking spot
  JUNCTION_AHEAD = 4,               //< mission control detected traffic sign and assumes a junction is ahead
  JUNCTION_AHEAD_AGAIN = 5,         //< mission control tries to search junction again after failed first try
  AFTER_JUNCTION = 6,               //< mission control is trying to locate road after junction
  DETERMINING_PARKING_SPOT = 7      //< DrivePullOut triggers vision to determine parking spot type
};

enum ObstacleSource : u_int8_t {
  IR_FRONT_CENTER_LONG,
  IR_FRONT_CENTER_SHORT,
  IR_FRONT_LEFT_LONG,
  IR_FRONT_LEFT_SHORT,
  IR_FRONT_RIGHT_LONG,
  IR_FRONT_RIGHT_SHORT,
  IR_REAR_LEFT_SHORT,
  IR_REAR_RIGHT_SHORT,
  IR_REAR_CENTER_SHORT,
  US_FRONT_LEFT,
  US_FRONT_RIGHT,
  US_REAR_LEFT,
  US_REAR_RIGHT,
  XTION = 13
};

// Action for communication with jury module
enum JuryAction : int8_t {
  STOP=-1,
  REQUEST_READY=0,
  RUN=1
};

enum SendAction : int8_t {
  ERROR=-1,
  READY=0,
  RUNNING=1,
  COMPLETE=2
};

//! Mission control status
enum MissionControlStatus : int8_t
{
  DRIVING_FORWARD,
  DRIVING_REVERSE,
  RESET,
  DUMMY
};

} //ns

#endif // KATANA_COMMON_INCLUDE
