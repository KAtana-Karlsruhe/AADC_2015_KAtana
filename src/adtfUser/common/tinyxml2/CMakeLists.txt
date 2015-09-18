cmake_minimum_required(VERSION 2.8.3)

set(LIBRARY_NAME    tinyxml2)

include_directories(.)

set(SOURCE_FILES
  tinyxml2.cpp
)

set(HEADER_FILES
  tinyxml2.h
)

set(KATANA_TINYXML_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/." CACHE INTERNAL "")


add_library(${LIBRARY_NAME} STATIC ${SOURCE_FILES} )

install(FILES ${HEADER_FILES}
        DESTINATION include)
install(TARGETS ${LIBRARY_NAME} 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/export)
