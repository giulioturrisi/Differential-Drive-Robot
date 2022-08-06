cmake_minimum_required(VERSION 3.5)
project(sim_ros2_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
    # fix C1128: number of sections exceeded object file format limit
    add_compile_options(/bigobj)
endif()

if(WIN32)
    # fix LNK1104: cannot open file 'libboost_regex-vc142-mt-x64-1_74.lib'
    set(Boost_USE_STATIC_LIBS ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pendulum_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(image_transport)

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        message(FATAL_ERROR "Environment variable COPPELIASIM_ROOT_DIR is not set")
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Boost REQUIRED)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim_ros2_interface)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include/sim_ros2_interface)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/templates/ ${CMAKE_CURRENT_SOURCE_DIR}/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" COMMAND ${Python3_EXECUTABLE} "${LIBPLUGIN_DIR}/simStubsGen/external/pycpp/pycpp.py" -p "interfaces_file=${CMAKE_CURRENT_SOURCE_DIR}/meta/interfaces.txt" -i "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}" -o "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" -P "${CMAKE_CURRENT_SOURCE_DIR}/tools" DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/parse_interfaces.py" "${CMAKE_CURRENT_SOURCE_DIR}/meta/interfaces.txt" "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}")
    set(generatedFiles ${generatedFiles} "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_ros_code DEPENDS ${generatedFiles})
coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/meta/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simROS2.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/sim_ros2_interface/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/sim_ros2_interface/config.h)

set(SOURCES
    src/sim_ros2_interface.cpp
    src/ros_msg_builtin_io.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/generated/callbacks.cpp
)
coppeliasim_add_plugin(simExtROS2 SOURCES ${SOURCES})
add_dependencies(simExtROS2 generate_ros_code)
target_include_directories(simExtROS2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(simExtROS2 Boost::boost)
#ament_target_dependencies(simExtROS2 rclcpp)
ament_target_dependencies(
    simExtROS2
    "diagnostic_msgs"
    "geometry_msgs"
    "lifecycle_msgs"
    "map_msgs"
    "nav_msgs"
    "pendulum_msgs"
    "rosgraph_msgs"
    "sensor_msgs"
    "shape_msgs"
    "std_msgs"
    "stereo_msgs"
    "tf2_geometry_msgs"
    "tf2_msgs"
    "trajectory_msgs"
    "unique_identifier_msgs"
    "visualization_msgs"
    "std_srvs"
    "example_interfaces"
)
if(image_transport_FOUND)
    ament_target_dependencies(
        simExtROS2
        "image_transport"
    )
endif()

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(simExtROS2 PRIVATE "SIM_ROS2_INTERFACE_BUILDING_LIBRARY")

ament_package()
