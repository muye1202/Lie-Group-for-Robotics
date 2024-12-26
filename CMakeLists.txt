cmake_minimum_required(VERSION 3.10)

# Set the project name and C++ standard
project(test VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen3 (ensure Eigen is installed on your system)
find_package(Eigen3 3.4 REQUIRED)

# Add your source files and header files
set(SOURCES
    test.cpp
    geometry/so3.cpp
    geometry/se3.cpp
)

set(HEADERS
    geometry/so3.h
    geometry/se3.h
)

# Include directories for the header files
include_directories(include)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link Eigen to the project
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

# Optionally, add compile definitions for debugging or optimization
target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_MODE)

# Optionally, set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
