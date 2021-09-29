add_executable(main_test main.cpp)

target_link_libraries(main_test PUBLIC robotino4lib)

target_include_directories(main_test
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")