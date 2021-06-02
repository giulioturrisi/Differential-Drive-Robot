cmake_minimum_required(VERSION 3.5)
project(driver_dc_l298N)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread  -lrt)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -pthread  -lrt)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")






include_directories(pigpio_INCLUDE_DIRS)




set(SOURCES test.cpp rotary_encoder.cpp)
add_executable(dc_motor ${SOURCES})

set (PROJECT_LINK_LIBS /usr/local/lib/libpigpio.so)
target_link_libraries(dc_motor ${PROJECT_LINK_LIBS})




