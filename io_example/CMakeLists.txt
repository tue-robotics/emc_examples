cmake_minimum_required(VERSION 2.8)
project(io_example)

add_executable( print_odom src/odom.cpp)
target_link_libraries(print_odom emc-framework)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
