add_library(robotino4 robotino4.cpp robotino4.hpp)

set_target_properties(robotino4 PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      SOVERSION ${PROJECT_VERSION_MAJOR}
                      PUBLIC_HEADER robotino4.hpp
                      DEBUG_POSTFIX d)

if(NOT DEFINED ROBOTINOAPI2_DIR)
        if(WIN32)
                if(DEFINED ENV{ROBOTINOAPI2_64_DIR})
                        string(REPLACE "\\" "/" ROBOTINOAPI2_DIR $ENV{ROBOTINOAPI2_64_DIR})
                else()
                        message(FATAL_ERROR "Environment variable ROBOTINOAPI2_64_DIR or ROBOTINOAPI2_DIR variable is not defined.")
                endif()
        else(WIN32)
                set(ROBOTINOAPI2_DIR "/opt/robotino")
        endif(WIN32)
endif(NOT DEFINED ROBOTINOAPI2_DIR)
set(CMAKE_MODULE_PATH "${ROBOTINOAPI2_DIR}/cmake")

find_package(RobotinoAPI2 REQUIRED)

# Create Robotino API2 IMPORTED target manually
# source: https://stackoverflow.com/questions/62000801/cmake-how-do-i-avoid-getting-absolute-paths-in-interface-target-properties
add_library(REC_RobotinoAPI2 SHARED IMPORTED)
if(WIN32)
        set_target_properties(REC_RobotinoAPI2 PROPERTIES
                              IMPORTED_LOCATION_RELEASE "${ROBOTINOAPI2_RELEASE_DLLS}"
                              IMPORTED_LOCATION_DEBUG "${ROBOTINOAPI2_DEBUG_DLLS}"
                              INTERFACE_INCLUDE_DIRECTORIES "${ROBOTINOAPI2_INCLUDE_DIR}"
                              IMPORTED_IMPLIB_RELEASE "${REC_ROBOTINO_API2_RELEASE_LIBRARY}"
                              IMPORTED_IMPLIB_DEBUG "${REC_ROBOTINO_API2_DEBUG_LIBRARY}"
                              IMPORTED_IMPLIB_RELWITHDEBINFO "${REC_ROBOTINO_API2_RELEASE_LIBRARY}"
                              IMPORTED_IMPLIB_MINSIZEREL "${REC_ROBOTINO_API2_RELEASE_LIBRARY}")
else(WIN32)
        set_target_properties(REC_RobotinoAPI2 PROPERTIES
                              IMPORTED_LOCATION "${REC_ROBOTINO_API2_LIBRARY}"
                              INTERFACE_INCLUDE_DIRECTORIES "${ROBOTINOAPI2_INCLUDE_DIR}")
endif(WIN32)

target_link_libraries(robotino4 PUBLIC REC_RobotinoAPI2)

target_include_directories(robotino4 INTERFACE
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:include>)

install(TARGETS robotino4
        EXPORT Robotino4WrapperTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT lib)