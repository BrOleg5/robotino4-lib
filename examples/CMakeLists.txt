add_executable(move_forward move_forward.cpp)

target_link_libraries(move_forward PUBLIC robotino4)

target_include_directories(move_forward PUBLIC ${CMAKE_SOURCE_DIR}/include)

install(TARGETS move_forward
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)