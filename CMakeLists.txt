cmake_minimum_required(VERSION 3.0)

set(ROBOTINO4WRAPPER_MAJOR_VERSION 1)
set(ROBOTINO4WRAPPER_MINOR_VERSION 3)
set(ROBOTINO4WRAPPER_PATCH_VERSION 0)
set(ROBOTINO4WRAPPER_VERSION ${ROBOTINO4WRAPPER_MAJOR_VERSION}.${ROBOTINO4WRAPPER_MINOR_VERSION}.${ROBOTINO4WRAPPER_PATCH_VERSION})

project(Robotino4Wrapper VERSION ${ROBOTINO4WRAPPER_VERSION} DESCRIPTION "Wrapper for library of Festo Robotino 4")

# set cmake config install dirs
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR cmake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/Robotino4Wrapper)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

include(GNUInstallDirs)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

option(BUILD_EXAMPLES "Build examples." OFF)

add_subdirectory(src)

if(BUILD_EXAMPLES)
        add_subdirectory(examples)
endif()

# Add all targets to the build-tree export set
export(TARGETS robotino4
        FILE "${PROJECT_BINARY_DIR}/cmake/Robotino4WrapperTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE Robotino4Wrapper)

# Create the Robotino4WrapperConfig.cmake and Robotino4WrapperConfigVersion.cmake files
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(cmake/Robotino4WrapperConfig.cmake.in
               "${PROJECT_BINARY_DIR}/cmake/Robotino4WrapperConfig.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${ROBOTINO4WRAPPER_CMAKE_DIR}/${CMAKE_INSTALL_INCLUDEDIR}")
configure_file(cmake/Robotino4WrapperConfig.cmake.in
               "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake/Robotino4WrapperConfig.cmake" @ONLY)

# ... for both
configure_file(cmake/Robotino4WrapperConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/cmake/Robotino4WrapperConfigVersion.cmake" @ONLY)

# Install the Robotino4WrapperConfig.cmake and Robotino4WrapperConfigVersion.cmake files
install(FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake/Robotino4WrapperConfig.cmake"
        "${PROJECT_BINARY_DIR}/cmake/Robotino4WrapperConfigVersion.cmake"
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT Robotino4WrapperTargets
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

include(InstallRequiredSystemLibraries)
set(CPACK_PACKAGE_VERSION_MAJOR "${ROBOTINO4WRAPPER_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${ROBOTINO4WRAPPER_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${ROBOTINO4WRAPPER_PATCH_VERSION}")
set(CPACK_SOURCE_GENERATOR "7Z")
include(CPack)