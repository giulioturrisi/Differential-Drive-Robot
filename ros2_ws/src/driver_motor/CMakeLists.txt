cmake_minimum_required(VERSION 3.5)
project(driver_motor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (required by rclcpp)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
find_package(pigpio REQUIRED)
include_directories(${pigpio_INCLUDE_DIRS})
include_directories(include)

set(SOURCES src/motor.cpp include/rotary_encoder.cpp)
add_executable(dc_motor ${SOURCES})

target_link_libraries(dc_motor ${pigpio_LIBRARY})
ament_target_dependencies(dc_motor rclcpp std_msgs geometry_msgs)

install(TARGETS
  dc_motor
  DESTINATION lib/${PROJECT_NAME})



ament_package()
