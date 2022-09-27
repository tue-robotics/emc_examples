cmake_minimum_required(VERSION 2.8)
project(my_project)
add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)

add_executable(example src/example.cpp src/map_visualization.h src/geometric_primitives.h)
include_directories(opencv)
target_link_libraries(example emc-framework ${OpenCV_LIBS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
