# find OpenCV
find_package( OpenCV REQUIRED )
find_package( ADTF REQUIRED )

# add the include directory of the filter
include_directories( ${OpenCV_INCLUDE_DIR})

# katana geometry
include_directories( ${KATANA_GEOMETRY_INCLUDE_DIRS} )

# declare our filter
adtf_add_filter(xtion_obstacle_detection_filter
    stdafx.h
    stdafx.cpp

    obstactle_detection.h
    obstactle_detection.cpp

    IPM.h
    IPM.cpp

)

# linking libs
target_link_libraries(xtion_obstacle_detection_filter ${OpenCV_LIBS} katanaGeometry)

adtf_install_plugin(xtion_obstacle_detection_filter ${AADC_INSTALL_DIR}/install_car)
