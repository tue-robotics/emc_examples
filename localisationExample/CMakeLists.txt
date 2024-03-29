# CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project("example")

# find python libraries
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)

include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})
include_directories({CMAKE_CURRENT_SOURCE_DIR}/include)

set(DCMAKE_BUILD_TYPE,"Release")

# add executable

add_executable(main src/main.cpp ./src/World.cpp ./src/LandMark.cpp ./src/Object.cpp 
               ./src/Robot.cpp ./src/Particle.cpp ./src/ParticleFilter.cpp ./src/ParticleFilterBase.cpp 
               ./src/Resampler.cpp ./src/AdaptiveParticleFilter.cpp)

# link python and numpy
target_link_libraries(main
    PRIVATE
        ${PYTHON_LIBRARIES}
        Python3::NumPy
)   

target_include_directories(main PUBLIC include/)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)