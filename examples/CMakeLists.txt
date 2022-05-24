add_executable(move_forward move_forward.cpp)

target_link_libraries(move_forward PUBLIC robotino4)

if(BUILD_EXAMPLES)
        install(TARGETS move_forward
                EXPORT Robotino4WrapperTargets
                RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/examples" COMPONENT bin)
endif()