cmake_minimum_required(VERSION 2.4)
project(markerDetection)

set(FILTER_NAME marker_detection)

find_package(ADTF REQUIRED)

#set include directories
include_directories(${ARUCO_INCLUDE_DIR}
    ${KATANA_INCLUDE_DIRS}
    ${KATANA_WORLD_INCLUDE_DIRS}
    ${oadrive_INCLUDE_DIRS}
)

# declare our filter
adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp
    MarkerDetection.h
    MarkerDetection.cpp
    #SignQueue.h
    #SignQueue.cpp
)

# linking libs
target_link_libraries(${FILTER_NAME}
  ${ARUCO_LIBS}
  ${OpenCV_LIBS}
  ${oadrive_LIBRARIES}
  )

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})