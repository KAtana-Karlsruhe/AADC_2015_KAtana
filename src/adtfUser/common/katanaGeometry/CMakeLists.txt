cmake_minimum_required(VERSION 2.8.3)

set(LIBRARY_NAME    katanaGeometry)

include_directories(
 include
 ${KATANA_INCLUDE_DIRS}
)

set(SOURCE_FILES
  src/Angle.cpp
  src/ExtendedPoint.cpp
  src/Line.cpp
  src/Point.cpp
  src/Pose.cpp
  src/Trajectory.cpp
  src/PoseStamped.cpp
  src/TrajectoryInterpolator.cpp
)

set(HEADER_FILES
  include/Angle.h
  include/ExtendedPoint.h
  include/Line.h
  include/Point.h
  include/Pose.h
  include/PoseStamped.h
  include/Trajectory.h
  include/TrajectoryInterpolator.h
)

set(KATANA_GEOMETRY_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include" CACHE INTERNAL "")


add_library(${LIBRARY_NAME} STATIC ${SOURCE_FILES} )
target_link_libraries(${LIBRARY_NAME} CGAL)

install (FILES ${HEADER_FILES}        
        DESTINATION include)
install(TARGETS ${LIBRARY_NAME} 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/export)
