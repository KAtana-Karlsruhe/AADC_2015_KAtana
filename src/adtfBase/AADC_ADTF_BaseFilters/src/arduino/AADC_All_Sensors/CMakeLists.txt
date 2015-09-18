set(FILTER_NAME aadc_GenericBundle)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp
                stdafx.h
                factory.cpp
                AbstractSensorBundle.h
                AbstractSensorBundle.cpp
                GyroSensorBundle.h
                GyroSensorBundle.cpp
                USSSensorBundle.h
                USSSensorBundle.cpp
                IRSensorBundle.h
                IRSensorBundle.cpp
                SteerangleSensorBundle.h
                SteerangleSensorBundle.cpp
                AccelSensorBundle.h
                AccelSensorBundle.cpp
                WheelcounterSensorBundle.h
                WheelcounterSensorBundle.cpp
                VoltageSensorBundle.h
                VoltageSensorBundle.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcbase/arduino) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
