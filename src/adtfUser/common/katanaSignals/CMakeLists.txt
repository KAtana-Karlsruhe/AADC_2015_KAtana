cmake_minimum_required(VERSION 2.8.3)

set(LIBRARY_NAME    katanaSignals)

include_directories(include)

set(SOURCE_FILES
)

set(HEADER_FILES
  include/SignalAverage.h
  include/SignalVariance.h
  include/SignalQueue.h
)

set(KATANA_SIGNALS_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include" CACHE INTERNAL "")


#add_library(${LIBRARY_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES} )

install (FILES ${HEADER_FILES}        
        DESTINATION include)
#install(TARGETS ${LIBRARY_NAME}
#        DESTINATION ${CMAKE_INSTALL_PREFIX}/export)
