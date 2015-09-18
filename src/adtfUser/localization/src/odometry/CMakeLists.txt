cmake_minimum_required(VERSION 2.4)
project(odometry)

set(FILTER_NAME odometry)

find_package(ADTF REQUIRED)

add_subdirectory(src)
