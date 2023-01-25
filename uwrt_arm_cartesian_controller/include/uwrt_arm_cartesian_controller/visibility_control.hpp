#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UWRT_ARM_CARTESIAN_CONTROLLER_EXPORT __attribute__((dllexport))
#define UWRT_ARM_CARTESIAN_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define UWRT_ARM_CARTESIAN_CONTROLLER_EXPORT __declspec(dllexport)
#define UWRT_ARM_CARTESIAN_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef UWRT_ARM_CARTESIAN_CONTROLLER_BUILDING_LIBRARY
#define UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC UWRT_ARM_CARTESIAN_CONTROLLER_EXPORT
#else
#define UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC UWRT_ARM_CARTESIAN_CONTROLLER_IMPORT
#endif
#define UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC_TYPE UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
#define UWRT_ARM_CARTESIAN_CONTROLLER_LOCAL
#else
#define UWRT_ARM_CARTESIAN_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define UWRT_ARM_CARTESIAN_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define UWRT_ARM_CARTESIAN_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
#define UWRT_ARM_CARTESIAN_CONTROLLER_LOCAL
#endif
#define UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC_TYPE
#endif
