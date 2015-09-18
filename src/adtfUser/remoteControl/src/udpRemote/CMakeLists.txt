set(FILTER_NAME udpRemote)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp	 
                stdafx.h
                udpRemote.h
                udpRemote.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
