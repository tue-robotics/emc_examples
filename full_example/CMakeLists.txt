cmake_minimum_required(VERSION 2.8)
project(my_project)

add_executable(full_example src/main.cpp
    src/worldModel.cpp
    src/detection.cpp
    src/driveControl.cpp)
target_link_libraries(full_example emc-framework)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
