if(NOT CMAKE_CL_64)
    set(FILTER_NAME saveImg)

    find_package(OpenCV REQUIRED)
    #set include directory
    include_directories(${ZBAR_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})


    # add the source
    adtf_add_filter(${FILTER_NAME}
                    src/saveImg.cpp  
                    src/saveImg.h
    )

    # linking libs
    target_link_libraries(${FILTER_NAME} ${ZBAR_LIBS} ${OpenCV_LIBS})

    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} aadcuser/common) 

    # Specify where it should be installed to
    adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
endif(NOT CMAKE_CL_64)	
