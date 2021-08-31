add_library(robotino4lib robotino4.cpp)

target_link_libraries(robotino4lib 
            /opt/robotino/lib/librec_robotino_api2.so
            )

target_include_directories(robotino4lib
          INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
          PUBLIC "/opt/robotino/include"
          "${CMAKE_CURRENT_SOURCE_DIR}/../include"
          )