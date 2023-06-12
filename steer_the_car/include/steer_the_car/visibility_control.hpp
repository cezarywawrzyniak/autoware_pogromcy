// Copyright 2023 Cezary_Wawrzyniak
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef STEER_THE_CAR__VISIBILITY_CONTROL_HPP_
#define STEER_THE_CAR__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(STEER_THE_CAR_BUILDING_DLL) || defined(STEER_THE_CAR_EXPORTS)
    #define STEER_THE_CAR_PUBLIC __declspec(dllexport)
    #define STEER_THE_CAR_LOCAL
  #else  // defined(STEER_THE_CAR_BUILDING_DLL) || defined(STEER_THE_CAR_EXPORTS)
    #define STEER_THE_CAR_PUBLIC __declspec(dllimport)
    #define STEER_THE_CAR_LOCAL
  #endif  // defined(STEER_THE_CAR_BUILDING_DLL) || defined(STEER_THE_CAR_EXPORTS)
#elif defined(__linux__)
  #define STEER_THE_CAR_PUBLIC __attribute__((visibility("default")))
  #define STEER_THE_CAR_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define STEER_THE_CAR_PUBLIC __attribute__((visibility("default")))
  #define STEER_THE_CAR_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // STEER_THE_CAR__VISIBILITY_CONTROL_HPP_
