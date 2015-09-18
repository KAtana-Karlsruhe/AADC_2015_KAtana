set(FILTER_NAME aadc_driver)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)

    set(HEADER_MOC
        displaywidget.h
        cDriverFilter.h
        )
    
    adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
    )
    
    adtf_add_filter(${FILTER_NAME}
                    stdafx.cpp
                    stdafx.h
					juryEnums.h
                    displaywidget.cpp
                    cDriverFilter.cpp
                    ${HEADER_MOC}
    )

    
    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} aadcbase/jury_modules) 
    
    adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
    
endif(QT_FOUND)



