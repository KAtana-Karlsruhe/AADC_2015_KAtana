cmake_minimum_required(VERSION 2.4)
project(obstacle_ir_us)

set(FILTER_NAME KATANA_obstacle_ir_us)

find_package(ADTF REQUIRED)

include_directories(${KATANA_GEOMETRY_INCLUDE_DIRS})
include_directories(${KATANA_SIGNALS_INCLUDE_DIRS})
include_directories(${KATANA_WORLD_INCLUDE_DIRS})
include_directories(${KATANA_INCLUDE_DIRS}
${oadrive_INCLUDE_DIRS})

# declare our filter
adtf_add_filter(${FILTER_NAME}
    filter.h
    filter.cpp
)

#linking
target_link_libraries(${FILTER_NAME} katanaGeometry katanaWorld)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
