#this files is not used in normal building process; it can be used if the whole project should be loaded with some IDE

cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

project(adtf_aadc_base_and_user)

#set the adtf dir
if(UNIX AND NOT ADTF_DIR)
    set(ADTF_DIR "/opt/adtf" CACHE PATH "ADTF Install Dir") 
endif(UNIX AND NOT ADTF_DIR)

#set the qmake executable
if(UNIX AND NOT QT_QMAKE_EXECUTABLE)
    set(QT_QMAKE_EXECUTABLE "/opt/qt-v4.7.1/bin/qmake" CACHE PATH "Qt Install Dir") 
endif(UNIX AND NOT QT_QMAKE_EXECUTABLE)


#set the install prefix
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../bin" CACHE PATH "The install directory" FORCE) 
    set(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT FALSE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
# can't be found and linked.
if(UNIX AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
            "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
            FORCE)
endif(UNIX AND NOT CMAKE_BUILD_TYPE)

# set toolchain file on Unix
if(UNIX AND NOT CMAKE_TOOLCHAINFILE)
    set(CMAKE_TOOLCHAIN_FILE "/opt/adtf/toolchainfiles/ARM_toolchain_build_on_target.cmake")
endif(UNIX AND NOT CMAKE_TOOLCHAINFILE)

set(AADC_BUILD_ALL_IN_ONE TRUE)

#include the source
add_subdirectory(adtfBase)
add_subdirectory(adtfUser)








