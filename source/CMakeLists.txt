cmake_minimum_required(VERSION 3.16)

# set the project name and version
project(robotino4lib VERSION 1.1 DESCRIPTION "Library for Festo Robotino 4")

add_subdirectory(tests)

include(GNUInstallDirs)

add_library(robotino4lib src/robotino4.cpp)

set_target_properties(robotino4lib PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER include/robotino4.hpp)

target_include_directories(robotino4lib PRIVATE include
                           PRIVATE src)

target_link_libraries(robotino4lib 
                      /opt/robotino/lib/librec_robotino_api2.so)
          
target_include_directories(robotino4lib
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC "/opt/robotino/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include")

install(TARGETS robotino4lib
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include)