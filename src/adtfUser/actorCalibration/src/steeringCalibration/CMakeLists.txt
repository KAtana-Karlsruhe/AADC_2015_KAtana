# add the include directory of the filter
include_directories(../include)
# declare our filter
adtf_add_filter(SteeringCalibration
    stdafx.h
    stdafx.cpp

    SteeringCalibration.h
    SteeringCalibration.cpp
)

adtf_install_plugin(SteeringCalibration ./)
