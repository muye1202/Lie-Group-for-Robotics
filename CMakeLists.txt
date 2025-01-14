cmake_minimum_required(VERSION 3.10)

# Set the project name and C++ standard
project(test VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen3 (ensure Eigen is installed on your system)
find_package(Eigen3 3.4 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Add your source files and header files
set(SOURCES
    diffdrive_example/odom.cpp
    lie_group/so3.cpp
    lie_group/se3.cpp
    lie_group/so2.cpp
    lie_group/se2.cpp
    lie_kinematics/kinematics.cpp
    lie_random/uncertainty.cpp
)

set(HEADERS
    lie_group/so2.h
    lie_group/se2.h
    lie_group/so3.h
    lie_group/se3.h
    lie_kinematics/kinematics.h
    lie_random/uncertainty.h
)

include_directories(
  ${CMAKE_SOURCE_DIR}/lie_group
  ${CMAKE_SOURCE_DIR}/lie_kinematics
  ${CMAKE_SOURCE_DIR}/lie_random
  ${CMAKE_SOURCE_DIR}/diffdrive_example
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link Eigen to the project
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json)

# Optionally, add compile definitions for debugging or optimization
target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_MODE)

# Optionally, set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
