cmake_minimum_required(VERSION 2.8)
project(io_example)

add_executable( echo_odom src/odom.cpp)
target_link_libraries(echo_odom emc-framework)

add_executable( echo_bumper src/bumper.cpp)
target_link_libraries(echo_bumper emc-framework)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
