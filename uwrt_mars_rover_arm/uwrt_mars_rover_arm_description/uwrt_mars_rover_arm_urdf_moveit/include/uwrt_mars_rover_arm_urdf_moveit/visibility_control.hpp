

#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UWRT_MOTION_PLANNING_EXPORT __attribute__((dllexport))
#define UWRT_MOTION_PLANNING_IMPORT __attribute__((dllimport))
#else
#define UWRT_MOTION_PLANNING_EXPORT __declspec(dllexport)
#define UWRT_MOTION_PLANNING_IMPORT __declspec(dllimport)
#endif
#ifdef UWRT_MOTION_PLANNING_BUILDING_LIBRARY
#define UWRT_MOTION_PLANNING_PUBLIC UWRT_MOTION_PLANNING_EXPORT
#else
#define UWRT_MOTION_PLANNING_PUBLIC UWRT_MOTION_PLANNING_IMPORT
#endif
#define UWRT_MOTION_PLANNING_PUBLIC_TYPE UWRT_MOTION_PLANNING_PUBLIC
#define UWRT_MOTION_PLANNING_LOCAL
#else
#define UWRT_MOTION_PLANNING_EXPORT __attribute__((visibility("default")))
#define UWRT_MOTION_PLANNING_IMPORT
#if __GNUC__ >= 4
#define UWRT_MOTION_PLANNING_PUBLIC __attribute__((visibility("default")))
#define UWRT_MOTION_PLANNING_LOCAL __attribute__((visibility("hidden")))
#else
#define UWRT_MOTION_PLANNING_PUBLIC
#define UWRT_MOTION_PLANNING_LOCAL
#endif
#define UWRT_MOTION_PLANNING_PUBLIC_TYPE
#endif
