add_library(robotino4 robotino4.cpp robotino4.hpp)

set_target_properties(robotino4 PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER robotino4.hpp
                      DEBUG_POSTFIX d)

if(WIN32 AND NOT CYGWIN)
        set(ROBOTINOAPI2_LIB "$ENV{ROBOTINOAPI2_64_DIR}/lib/rec_robotino_api2.lib")
        set(ROBOTINOAPI2_INCLUDE_DIR "$ENV{ROBOTINOAPI2_64_DIR}/include")
else()
        set(ROBOTINOAPI2_LIB /opt/robotino/lib/librec_robotino_api2.so)
        set(ROBOTINOAPI2_INCLUDE_DIR "/opt/robotino/include")
endif()

target_link_libraries(robotino4 PUBLIC ${ROBOTINOAPI2_LIB})

target_include_directories(robotino4 PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                           ${ROBOTINOAPI2_INCLUDE_DIR})

install(TARGETS robotino4
        EXPORT Robotino4WrapperTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT dev)