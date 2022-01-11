#ifndef UWRT_MARS_ROVER_UTILS__VISIBILITY_CONTROL_H_
#define UWRT_MARS_ROVER_UTILS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UWRT_MARS_ROVER_UTILS_EXPORT __attribute__ ((dllexport))
    #define UWRT_MARS_ROVER_UTILS_IMPORT __attribute__ ((dllimport))
  #else
    #define UWRT_MARS_ROVER_UTILS_EXPORT __declspec(dllexport)
    #define UWRT_MARS_ROVER_UTILS_IMPORT __declspec(dllimport)
  #endif
  #ifdef UWRT_MARS_ROVER_UTILS_BUILDING_LIBRARY
    #define UWRT_MARS_ROVER_UTILS_PUBLIC UWRT_MARS_ROVER_UTILS_EXPORT
  #else
    #define UWRT_MARS_ROVER_UTILS_PUBLIC UWRT_MARS_ROVER_UTILS_IMPORT
  #endif
  #define UWRT_MARS_ROVER_UTILS_PUBLIC_TYPE UWRT_MARS_ROVER_UTILS_PUBLIC
  #define UWRT_MARS_ROVER_UTILS_LOCAL
#else
  #define UWRT_MARS_ROVER_UTILS_EXPORT __attribute__ ((visibility("default")))
  #define UWRT_MARS_ROVER_UTILS_IMPORT
  #if __GNUC__ >= 4
    #define UWRT_MARS_ROVER_UTILS_PUBLIC __attribute__ ((visibility("default")))
    #define UWRT_MARS_ROVER_UTILS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UWRT_MARS_ROVER_UTILS_PUBLIC
    #define UWRT_MARS_ROVER_UTILS_LOCAL
  #endif
  #define UWRT_MARS_ROVER_UTILS_PUBLIC_TYPE
#endif

#endif  // UWRT_MARS_ROVER_UTILS__VISIBILITY_CONTROL_H_
