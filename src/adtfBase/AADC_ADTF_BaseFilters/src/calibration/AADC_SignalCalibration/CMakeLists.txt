set(FILTER_NAME aadc_sensorScaling)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                 stdafx.cpp
                 stdafx.h
                 SensorScaling.h
                 SensorScaling.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcbase/calibration) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
