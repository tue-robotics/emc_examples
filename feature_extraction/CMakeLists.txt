cmake_minimum_required(VERSION 2.8)
project(feature_extraction)

find_package(OpenCV 3 REQUIRED)

add_executable( main src/main.cpp
                src/featureExtraction.cpp
                src/featureExtraction.h )

target_link_libraries(main emc-framework)
target_link_libraries(main ${OpenCV_LIBS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
