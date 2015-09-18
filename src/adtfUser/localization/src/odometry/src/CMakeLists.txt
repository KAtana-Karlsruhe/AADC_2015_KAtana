include_directories(${KATANA_GEOMETRY_INCLUDE_DIRS})
include_directories(${KATANA_SIGNALS_INCLUDE_DIRS})
include_directories(${KATANA_INCLUDE_DIRS})
include_directories(${KATANA_WORLD_INCLUDE_DIRS}
                    ${oadrive_INCLUDE_DIRS})

# declare our filter
adtf_add_filter(${FILTER_NAME}
    odometry_filter.h
    odometry_filter.cpp

    stdafx.h
    stdafx.cpp
)

#linking
target_link_libraries(${FILTER_NAME} katanaGeometry)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
