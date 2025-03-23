cmake_minimum_required(VERSION 3.10)

# Set the project name and C++ standard
project(lie_lib VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen3 (ensure Eigen is installed on your system)
find_package(Eigen3 3.4 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Add your source files and header files
set(SOURCES
    diffdrive_example/odom.cpp
    src/lie_group/so3.cpp
    src/lie_group/se3.cpp
    src/lie_group/so2.cpp
    src/lie_group/se2.cpp
    src/lie_kinematics/kinematics.cpp
    src/lie_random/uncertainty.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Add demo executable
add_executable(demo_lie_lib diffdrive_example/odom.cpp)
target_link_libraries(demo_lie_lib PRIVATE 
                      ${PROJECT_NAME} 
                      Eigen3::Eigen 
                      nlohmann_json::nlohmann_json)


target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link Eigen to the project
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json)

# Optionally, add compile definitions for debugging or optimization
target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_MODE)

# Optionally, set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)

# Installation rules
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/lie_lib DESTINATION include)

include(CMakePackageConfigHelpers)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)
