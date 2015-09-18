cmake_minimum_required(VERSION 2.8.3)
set(FILTER_NAME katana_mission_control)

include_directories(
  ${KATANA_MISSION_CONTROL_INCLUDE_DIRS}
  ${KATANA_INCLUDE_DIRS}
  ${KATANA_TINYXML_INCLUDE_DIRS}
  ${KATANA_WORLD_INCLUDE_DIRS}
  ${KATANA_GEOMETRY_INCLUDE_DIRS}
  ${KATANA_SIGNALS_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
)

# declare our filter
adtf_add_filter(${FILTER_NAME}
  filter.h
  filter.cpp
)

target_link_libraries(${FILTER_NAME}
  missionControl
  tinyxml2
  ${oadrive_LIBRARIES}
)

# Dependencies
add_dependencies(${FILTER_NAME}
  mission_control
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
