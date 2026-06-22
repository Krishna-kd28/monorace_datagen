cmake_minimum_required(VERSION 3.8)

project(mav_description)

# Default to C99 and C++14
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(DIRECTORY
  launch
  models
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
