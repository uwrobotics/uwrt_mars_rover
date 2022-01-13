// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UWRT_MARS_ROVER_ARM_HW__VISIBILITY_H_
#define UWRT_MARS_ROVER_ARM_HW__VISIBILITY_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define UWRT_MARS_ROVER_ARM_HW_EXPORT __attribute__((dllexport))
#define UWRT_MARS_ROVER_ARM_HW_IMPORT __attribute__((dllimport))
#else
#define UWRT_MARS_ROVER_ARM_HW_EXPORT __declspec(dllexport)
#define UWRT_MARS_ROVER_ARM_HW_IMPORT __declspec(dllimport)
#endif

#ifdef UWRT_MARS_ROVER_ARM_HW_DLL
#define UWRT_MARS_ROVER_ARM_HW_PUBLIC UWRT_MARS_ROVER_ARM_HW_EXPORT
#else
#define UWRT_MARS_ROVER_ARM_HW_PUBLIC UWRT_MARS_ROVER_ARM_HW_IMPORT
#endif

#define UWRT_MARS_ROVER_ARM_HW_PUBLIC_TYPE UWRT_MARS_ROVER_ARM_HW_PUBLIC

#define UWRT_MARS_ROVER_ARM_HW_LOCAL

#else

#define UWRT_MARS_ROVER_ARM_HW_EXPORT __attribute__((visibility("default")))
#define UWRT_MARS_ROVER_ARM_HW_IMPORT

#if __GNUC__ >= 4
#define UWRT_MARS_ROVER_ARM_HW_PUBLIC __attribute__((visibility("default")))
#define UWRT_MARS_ROVER_ARM_HW_LOCAL __attribute__((visibility("hidden")))
#else
#define UWRT_MARS_ROVER_ARM_HW_PUBLIC
#define UWRT_MARS_ROVER_ARM_HW_LOCAL
#endif

#define UWRT_MARS_ROVER_ARM_HW_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // UWRT_MARS_ROVER_ARM_HW__VISIBILITY_H_
