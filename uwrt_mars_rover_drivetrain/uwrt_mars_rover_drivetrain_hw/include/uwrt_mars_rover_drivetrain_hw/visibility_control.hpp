#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_EXPORT __attribute__((dllexport))
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_IMPORT __attribute__((dllimport))
#else
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_EXPORT __declspec(dllexport)
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_IMPORT __declspec(dllimport)
#endif
#ifdef UWRT_MARS_ROVER_DRIVETRAIN_HW_BUILDING_LIBRARY
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC UWRT_MARS_ROVER_DRIVETRAIN_HW_EXPORT
#else
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC UWRT_MARS_ROVER_DRIVETRAIN_HW_IMPORT
#endif
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC_TYPE UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
#else
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_EXPORT __attribute__((visibility("default")))
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_IMPORT
#if __GNUC__ >= 4
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC __attribute__((visibility("default")))
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL __attribute__((visibility("hidden")))
#else
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
#endif
#define UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC_TYPE
#endif
