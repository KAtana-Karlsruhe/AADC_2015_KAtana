cmake_minimum_required(VERSION 2.4)
project(odometrySimulator)

set(FILTER_NAME odometrySimulator)

find_package(ADTF REQUIRED)

add_subdirectory(src)
