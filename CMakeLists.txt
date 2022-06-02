cmake_minimum_required(VERSION 3.0)

set(ROBOTINO4WRAPPER_MAJOR_VERSION 1)
set(ROBOTINO4WRAPPER_MINOR_VERSION 4)
set(ROBOTINO4WRAPPER_PATCH_VERSION 2)
set(ROBOTINO4WRAPPER_VERSION ${ROBOTINO4WRAPPER_MAJOR_VERSION}.${ROBOTINO4WRAPPER_MINOR_VERSION}.${ROBOTINO4WRAPPER_PATCH_VERSION})

project(Robotino4Wrapper VERSION ${ROBOTINO4WRAPPER_VERSION} DESCRIPTION "Wrapper for library of Festo Robotino 4")

include(GNUInstallDirs)

option(BUILD_EXAMPLES "Build examples." OFF)

add_subdirectory(src)

if(BUILD_EXAMPLES)
        add_subdirectory(examples)
endif()

# Install the export set for use with the install-tree
install(EXPORT Robotino4WrapperTargets
        FILE Robotino4WrapperTargets.cmake
        DESTINATION "cmake" COMPONENT lib)

include(CMakePackageConfigHelpers)

# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
                              "${CMAKE_CURRENT_BINARY_DIR}/cmake/Robotino4WrapperConfig.cmake"
                              INSTALL_DESTINATION "cmake"
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/cmake/Robotino4WrapperConfigVersion.cmake"
                                 VERSION "${ROBOTINO4WRAPPER_VERSION}"
                                 COMPATIBILITY AnyNewerVersion)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/cmake/Robotino4WrapperConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake/Robotino4WrapperConfigVersion.cmake"
        DESTINATION "cmake" COMPONENT lib)

export(EXPORT Robotino4WrapperTargets
       FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/Robotino4WrapperTargets.cmake")

include(InstallRequiredSystemLibraries)
set(CPACK_PACKAGE_VERSION_MAJOR "${ROBOTINO4WRAPPER_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${ROBOTINO4WRAPPER_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${ROBOTINO4WRAPPER_PATCH_VERSION}")

if(NOT WIN32)
        set(CPACK_DEBIAN_PACKAGE_MAINTAINER "oleg.brylyov@mail.ru")
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "robotino-api2 (>= 1.1.14), robotino-dev (>= 1.0.3)")
        set(CPACK_PROJECT_CONFIG_FILE "${CMAKE_SOURCE_DIR}/cmake/PackageConfig.txt")
endif(NOT WIN32)

if(WIN32)
        set(CPACK_SOURCE_GENERATOR "ZIP;7Z")
elseif(WIN32)
        set(CPACK_SOURCE_GENERATOR "TGZ")
endif(WIN32)
set(CPACK_SOURCE_IGNORE_FILES "/\\\\.git/;/\\\\.vscode/;/\\\\build/;/\\\\builds/")

include(CPack)

cpack_add_component(lib DISPLAY_NAME "Library"
                    REQUIRED)
cpack_add_component(examples DISPLAY_NAME "Examples")