cmake_minimum_required(VERSION 2.8.3)
set(LIBRARY_NAME    helperClasses)

include_directories(
  ${KATANA_MISSION_CONTROL_INCLUDE_DIRS}
  ${KATANA_INCLUDE_DIRS}
  ${KATANA_WORLD_INCLUDE_DIRS}
  ${KATANA_TINYXML_INCLUDE_DIRS}
  ${KATANA_GEOMETRY_INCLUDE_DIRS}
  ${KATANA_SIGNALS_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS})

#HEADER
set(HELPER_CLASSES_HEADERS
  Config.h
  HelperFunctions.h
  WorldPlausibilityChecker.h
)

#SOURCES
set(HELPER_CLASSES_SOURCES
  Config.cpp
  HelperFunctions.cpp
  WorldPlausibilityChecker.cpp
)



add_library(${LIBRARY_NAME} STATIC ${HELPER_CLASSES_SOURCES} )

target_link_libraries(${LIBRARY_NAME}
        katanaGeometry
        tinyxml2
        missionControl
)

install(TARGETS ${LIBRARY_NAME}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/export)
