add_library(robotino4 robotino4.cpp robotino4.hpp)

if(WIN32)
        string(REPLACE "\\" "/" ROBOTINOAPI2_DIR $ENV{ROBOTINOAPI2_64_DIR})
        set(CMAKE_MODULE_PATH "${ROBOTINOAPI2_DIR}/cmake")
endif(WIN32)

find_package(RobotinoAPI2)

set_target_properties(robotino4 PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      SOVERSION ${PROJECT_VERSION_MAJOR}
                      PUBLIC_HEADER robotino4.hpp
                      DEBUG_POSTFIX d)

target_link_libraries(robotino4 PUBLIC ${REC_ROBOTINO_API2_LIBRARY})

target_include_directories(robotino4 INTERFACE
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:include>
                           PUBLIC ${ROBOTINOAPI2_INCLUDE_DIR})

install(TARGETS robotino4
        EXPORT Robotino4WrapperTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT dev)