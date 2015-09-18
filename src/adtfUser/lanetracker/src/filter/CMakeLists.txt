cmake_minimum_required(VERSION 2.8.3)
set(FILTER_NAME katana_lanetracker)

include_directories(
  ${KATANA_LANETRACKER_INCLUDE_DIRS}
  ${KATANA_INCLUDE_DIRS}
  ${KATANA_WORLD_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
)

# declare our filter
adtf_add_filter(${FILTER_NAME}
  Lanetracker.h
  Lanetracker.cpp
)

target_link_libraries(${FILTER_NAME}
  ${OpenCV_LIBS}
  ${oadrive_LIBRARIES}
)

# Dependencies
add_dependencies(${FILTER_NAME}
  lanetracker
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR}/install_car)
