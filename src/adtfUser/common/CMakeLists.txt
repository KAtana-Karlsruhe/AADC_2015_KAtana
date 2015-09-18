set(KATANA_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/katanaInclude" CACHE INTERNAL "")

add_subdirectory(tinyxml2)
add_subdirectory(katanaGeometry)
add_subdirectory(katanaSignals)
add_subdirectory(katanaWorld)
