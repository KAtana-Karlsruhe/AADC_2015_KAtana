cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(adtf_aadc_user)

    #set the install prefix if not already done.
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../bin" CACHE PATH "The install directory" FORCE)
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

endif(NOT AADC_BUILD_ALL_IN_ONE)

find_package(ADTF REQUIRED)
#-------General------------


set(AADC_INSTALL_DIR .)
set(AADC_INSTALL_DIR_LIBS .)

set(AADC_DIR ../)
find_package(AADC REQUIRED)

# Enable C++11 support in GNU-Compiler
IF(CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math -std=c++11")
ENDIF()

#-------------- oadrive -------------
find_package(oadrive REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
${EIGEN3_INCLUDE_DIR}
)
#--------------sources----------------------------
add_subdirectory(common)
add_subdirectory(demo)
add_subdirectory(remoteControl)
add_subdirectory(actorCalibration)
add_subdirectory(localization)
add_subdirectory(katana_mission_control)
add_subdirectory(lanetracker)
add_subdirectory(obstacleDetection)
add_subdirectory(markerdetection)