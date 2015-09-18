adtf_add_filter(DemoPatchGenerator
    stdafx.h
    stdafx.cpp

    DemoPatchGenerator.h
    DemoPatchGenerator.cpp
)

include_directories(
${oadrive_INCLUDE_DIRS}
)

adtf_add_filter(DemoObstacleGenerator
    stdafx.h
    stdafx.cpp

    DemoObstacleGenerator.h
    DemoObstacleGenerator.cpp
)

#linking
target_link_libraries(DemoPatchGenerator katanaGeometry ${oadrive_LIBRARIES})
target_link_libraries(DemoObstacleGenerator katanaGeometry)

adtf_install_plugin(DemoPatchGenerator ./)
adtf_install_plugin(DemoObstacleGenerator ./)
