cmake_minimum_required(VERSION 2.8)
project(full_example)

add_executable( main src/main.cpp
                src/worldModel.cpp
                src/worldModel.h
                src/detection.cpp
                src/detection.h
                src/driveControl.cpp
                src/driveControl.h
                src/config.h )

target_link_libraries(main emc-framework)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
