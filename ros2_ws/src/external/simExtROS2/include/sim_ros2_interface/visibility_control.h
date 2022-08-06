#ifndef SIM_ROS2_INTERFACE__VISIBILITY_CONTROL_H_
#define SIM_ROS2_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIM_ROS2_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define SIM_ROS2_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define SIM_ROS2_INTERFACE_EXPORT __declspec(dllexport)
    #define SIM_ROS2_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIM_ROS2_INTERFACE_BUILDING_LIBRARY
    #define SIM_ROS2_INTERFACE_PUBLIC SIM_ROS2_INTERFACE_EXPORT
  #else
    #define SIM_ROS2_INTERFACE_PUBLIC SIM_ROS2_INTERFACE_IMPORT
  #endif
  #define SIM_ROS2_INTERFACE_PUBLIC_TYPE SIM_ROS2_INTERFACE_PUBLIC
  #define SIM_ROS2_INTERFACE_LOCAL
#else
  #define SIM_ROS2_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define SIM_ROS2_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define SIM_ROS2_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define SIM_ROS2_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIM_ROS2_INTERFACE_PUBLIC
    #define SIM_ROS2_INTERFACE_LOCAL
  #endif
  #define SIM_ROS2_INTERFACE_PUBLIC_TYPE
#endif

#endif  // SIM_ROS2_INTERFACE__VISIBILITY_CONTROL_H_
