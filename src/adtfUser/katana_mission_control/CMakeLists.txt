include_directories(./include)

cmake_minimum_required(VERSION 2.8.3)
project(katana_mission_control)

set(FILTER_NAME katana_mission_control)

find_package(ADTF REQUIRED)

set(KATANA_MISSION_CONTROL_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/src" CACHE INTERNAL "")

add_subdirectory(src/filter)
add_subdirectory(src/mission_control)
add_subdirectory(src/helperClasses)

#test tools

#test_maneuver
add_executable(test_maneuver tools/test_maneuver.cpp ${MISSION_CONTROL_SOURCES} ${VICINITY_SOURCES} ${KATANA_MISSION_CONTROL_INCLUDE_DIRS})
include_directories(
  ${KATANA_MISSION_CONTROL_INCLUDE_DIRS}
  ${KATANA_INCLUDE_DIRS}
  ${KATANA_TINYXML_INCLUDE_DIRS}
  ${KATANA_WORLD_INCLUDE_DIRS}
  ${KATANA_SIGNALS_INCLUDE_DIRS}
  ${KATANA_GEOMETRY_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
)
target_link_libraries(test_maneuver missionControl tinyxml2 katanaWorld)

#test_geometry
add_executable(test_geometry tools/test_geometry.cpp)
include_directories(
  ${KATANA_GEOMETRY_INCLUDE_DIRS}
)
target_link_libraries(test_geometry katanaGeometry missionControl)

#test_world
add_executable(test_world tools/test_world.cpp ${MISSION_CONTROL_SOURCES} ${VICINITY_SOURCES} ${KATANA_MISSION_CONTROL_INCLUDE_DIRS})
include_directories(
  ${KATANA_MISSION_CONTROL_INCLUDE_DIRS}
  ${KATANA_INCLUDE_DIRS}
  ${KATANA_TINYXML_INCLUDE_DIRS}
  ${KATANA_WORLD_INCLUDE_DIRS}
  ${KATANA_SIGNALS_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
)
target_link_libraries(test_world missionControl tinyxml2 katanaWorld)

#test_position_controller
add_executable(test_position_controller tools/test_position_controller.cpp)
include_directories(
  ${KATANA_GEOMETRY_INCLUDE_DIRS}
  ${KATANA_MISSION_CONTROL_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
)
target_link_libraries(test_position_controller katanaGeometry missionControl tinyxml2 CGAL)

#test_obstacles
add_executable(test_obstacles tools/test_obstacles.cpp ${MISSION_CONTROL_SOURCES} ${VICINITY_SOURCES} ${KATANA_MISSION_CONTROL_INCLUDE_DIRS})
include_directories(
  ${KATANA_MISSION_CONTROL_INCLUDE_DIRS}
  ${KATANA_INCLUDE_DIRS}
  ${KATANA_TINYXML_INCLUDE_DIRS}
  ${KATANA_WORLD_INCLUDE_DIRS}
  ${KATANA_SIGNALS_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
)
target_link_libraries(test_obstacles missionControl tinyxml2 katanaWorld)


add_subdirectory(tools/demoDataGenerators)
