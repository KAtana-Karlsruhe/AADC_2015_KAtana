cmake_minimum_required(VERSION 2.8.3)

set(LIBRARY_NAME    katanaWorld)

include_directories(
  include
  ${KATANA_GEOMETRY_INCLUDE_DIRS}
  ${KATANA_INCLUDE_DIRS}
  ${KATANA_TINYXML_INCLUDE_DIRS}
  ${oadrive_INCLUDE_DIRS}
)

set(SOURCE_FILES
  src/RoadBase.cpp
  src/RoadJunction.cpp
  src/RoadPatch.cpp
  src/Obstacle.cpp
)

set(HEADER_FILES
  include/RoadBase.h
  include/RoadJunction.h
  include/RoadPatch.h
  include/Obstacle.h
  include/PinConversions.h
  include/PoseWithTime.h
)

set(KATANA_WORLD_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include" CACHE INTERNAL "")


add_library(${LIBRARY_NAME} STATIC ${SOURCE_FILES} )
target_link_libraries(${LIBRARY_NAME} tinyxml2 katanaGeometry ${oadrive_LIBRARIES})

install(FILES ${HEADER_FILES}
        DESTINATION include)
install(TARGETS ${LIBRARY_NAME} 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/export)
