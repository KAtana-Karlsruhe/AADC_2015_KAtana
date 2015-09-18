set(FILTER_NAME aadc_actuator)

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp
                stdafx.h
                actuatorfilter.h
                actuatorfilter.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcbase/arduino) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})


