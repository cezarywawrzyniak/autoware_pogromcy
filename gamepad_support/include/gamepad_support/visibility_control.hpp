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

#ifndef GAMEPAD_SUPPORT__VISIBILITY_CONTROL_HPP_
#define GAMEPAD_SUPPORT__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(GAMEPAD_SUPPORT_BUILDING_DLL) || defined(GAMEPAD_SUPPORT_EXPORTS)
    #define GAMEPAD_SUPPORT_PUBLIC __declspec(dllexport)
    #define GAMEPAD_SUPPORT_LOCAL
  #else  // defined(GAMEPAD_SUPPORT_BUILDING_DLL) || defined(GAMEPAD_SUPPORT_EXPORTS)
    #define GAMEPAD_SUPPORT_PUBLIC __declspec(dllimport)
    #define GAMEPAD_SUPPORT_LOCAL
  #endif  // defined(GAMEPAD_SUPPORT_BUILDING_DLL) || defined(GAMEPAD_SUPPORT_EXPORTS)
#elif defined(__linux__)
  #define GAMEPAD_SUPPORT_PUBLIC __attribute__((visibility("default")))
  #define GAMEPAD_SUPPORT_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define GAMEPAD_SUPPORT_PUBLIC __attribute__((visibility("default")))
  #define GAMEPAD_SUPPORT_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // GAMEPAD_SUPPORT__VISIBILITY_CONTROL_HPP_
