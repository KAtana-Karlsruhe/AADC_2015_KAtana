cmake_minimum_required(VERSION 2.8.3)
project(lanetracker)

set(FILTER_NAME lanetracker)

find_package(ADTF REQUIRED)

set(KATANA_LANETRACKER_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/src/filter" "${CMAKE_CURRENT_LIST_DIR}/src/lanetracker" CACHE INTERNAL "")

add_subdirectory(src/filter)
#add_subdirectory(src/lanetracker)

#test tools
#add_executable(ManagedPoseTest test/ManagedPoseTest.cpp ${LANETRACKER_SOURCES} ${KATANA_LANETRACKER_INCLUDE_DIRS})
#add_executable(ManagedPoseRotationTest test/ManagedPoseRotationTest.cpp ${LANETRACKER_SOURCES} ${KATANA_LANETRACKER_INCLUDE_DIRS})
#add_executable(MatcherTest test/MatcherTest.cpp ${LANETRACKER_SOURCES} ${KATANA_LANETRACKER_INCLUDE_DIRS})
#add_executable(StitcherTest test/StitcherTest.cpp ${LANETRACKER_SOURCES} ${KATANA_LANETRACKER_INCLUDE_DIRS})

include_directories(
#${KATANA_LANETRACKER_INCLUDE_DIRS}
${KATANA_INCLUDE_DIRS}
${KATANA_GEOMETRY_INCLUDE_DIRS}
${KATANA_WORLD_INCLUDE_DIRS}
)

#target_link_libraries(ManagedPoseTest lanetracker)
#target_link_libraries(ManagedPoseRotationTest lanetracker)
#target_link_libraries(MatcherTest lanetracker)
#target_link_libraries(StitcherTest lanetracker)

