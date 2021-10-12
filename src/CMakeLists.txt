add_library(Robotino4Lib robotino4.cpp robotino4.hpp)

set_target_properties(Robotino4Lib PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER robotino4.hpp)

target_link_libraries(Robotino4Lib PUBLIC /opt/robotino/lib/librec_robotino_api2.so)

target_include_directories(Robotino4Lib PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/Robotino4>
                           "/opt/robotino/include")

install(TARGETS Robotino4Lib
        EXPORT Robotino4Targets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/Robotino4" COMPONENT dev)