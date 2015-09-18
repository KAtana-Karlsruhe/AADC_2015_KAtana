include_directories(${KATANA_GEOMETRY_INCLUDE_DIRS})
include_directories(
${KATANA_SIGNALS_INCLUDE_DIRS}
${KATANA_INCLUDE_DIRS}
${KATANA_WORLD_INCLUDE_DIRS}
${oadrive_INCLUDE_DIRS})

# declare our filter
adtf_add_filter(${FILTER_NAME}
    filter.h
    filter.cpp

    stdafx.h
    stdafx.cpp
)

#linking
target_link_libraries(${FILTER_NAME} katanaGeometry ${oadrive_LIBRARIES})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
