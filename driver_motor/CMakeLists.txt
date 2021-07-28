cmake_minimum_required(VERSION 3.5)
project(driver_motor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
include_directories(pigpio_INCLUDE_DIRS)
include_directories(include)

set(SOURCES src/motor.cpp include/rotary_encoder.cpp)
add_executable(dc_motor ${SOURCES})

set (PROJECT_LINK_LIBS /usr/local/lib/libpigpio.so)
target_link_libraries(dc_motor ${PROJECT_LINK_LIBS})
ament_target_dependencies(dc_motor rclcpp tf2_msgs tf2 std_msgs)

install(TARGETS
  dc_motor
  DESTINATION lib/${PROJECT_NAME})



ament_package()
