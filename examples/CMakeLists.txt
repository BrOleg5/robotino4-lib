add_executable(move_forward move_forward.cpp)

target_link_libraries(move_forward PUBLIC robotino4)

install(TARGETS move_forward
        RUNTIME DESTINATION COMPONENT examples)