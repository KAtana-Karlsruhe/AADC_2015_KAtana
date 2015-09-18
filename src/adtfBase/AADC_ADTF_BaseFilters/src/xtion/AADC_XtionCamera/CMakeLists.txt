set(FILTER_NAME aadc_xtion)

# include directories
include_directories(${OPENNI2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR}) 

# add the source
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp
                stdafx.h
                xtioncapturefilter.h
                xtioncapturefilter.cpp 
                xtionGrabber.h
 )

# linking libs
target_link_libraries(${FILTER_NAME} ${OPENNI2_LIBS} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcbase/xtion) 

 # Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
