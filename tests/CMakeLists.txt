add_executable(Robotino4Test main.cpp)

target_link_libraries(Robotino4Test PUBLIC Robotino4Lib)

# target_include_directories(Robotino4Test
#         PUBLIC "${PROJECT_SOURCE_DIR}/src")

if(BUILD_TESTS)
        install(TARGETS Robotino4Test
                EXPORT Robotino4Targets
                RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/Robotino4Test" COMPONENT bin)
endif()