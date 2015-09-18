cmake_minimum_required(VERSION 2.8.3)
set(LIBRARY_NAME    missionControl)

include_directories(
  ${KATANA_MISSION_CONTROL_INCLUDE_DIRS}
  ${KATANA_INCLUDE_DIRS}
  ${KATANA_TINYXML_INCLUDE_DIRS}
  ${KATANA_WORLD_INCLUDE_DIRS}
  ${KATANA_SIGNALS_INCLUDE_DIRS}
  ${KATANA_GEOMETRY_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
)

#HEADER
set(MISSION_CONTROL_HEADERS
  DrivingStripChanger.h
  LanetrackerJobManager.h
  LateralController.h
  mission_control.h
  maneuver.h
  MotionPlanning.h
  ParkingAssistant.h
  PositionController.h
  RoadSection.h
  StateManager.h
  StateBase.h
  System.h
  VelocityControl.h
  world.h
  LightController.h
  CollisionHandler.h

  states/main_states/MainStateBase.h
  states/main_states/StateDriving.h
  states/main_states/StateWaiting.h
  states/main_states/StateError.h
  states/main_states/StateRecovering.h

  states/driving_states/DriveBase.h
  states/driving_states/DriveFinished.h
  states/driving_states/DriveNormal.h
  states/driving_states/DriveInitialize.h
  states/driving_states/DriveParking.h
  states/driving_states/DriveJunction.h
  states/driving_states/DrivePullOut.h
)

#SOURCES
set(MISSION_CONTROL_SOURCES
  DrivingStripChanger.cpp
  LanetrackerJobManager.cpp
  LateralController.cpp
  mission_control.cpp
  maneuver.cpp
  MotionPlanning.cpp
  ParkingAssistant.cpp
  PositionController.cpp
  RoadSection.cpp
  StateBase.cpp
  System.cpp
  VelocityControl.cpp
  world.cpp
  LightController.cpp
  CollisionHandler.cpp

  states/main_states/MainStateBase.cpp
  states/main_states/StateDriving.cpp
  states/main_states/StateWaiting.cpp
  states/main_states/StateError.cpp
  states/main_states/StateRecovering.cpp

  states/driving_states/DriveBase.cpp
  states/driving_states/DriveFinished.cpp
  states/driving_states/DriveNormal.cpp
  states/driving_states/DriveInitialize.cpp
  states/driving_states/DriveParking.cpp
  states/driving_states/DriveJunction.cpp
  states/driving_states/DrivePullOut.cpp
)



add_library(${LIBRARY_NAME} STATIC ${MISSION_CONTROL_SOURCES} )
target_link_libraries(${LIBRARY_NAME}
  ${oadrive_LIBRARIES}
  tinyxml2
  katanaWorld
  CGAL
  helperClasses
)

install(TARGETS ${LIBRARY_NAME}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/export)
