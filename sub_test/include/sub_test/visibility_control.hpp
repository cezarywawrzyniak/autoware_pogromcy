// Copyright 2023 borys
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

#ifndef SUB_TEST__VISIBILITY_CONTROL_HPP_
#define SUB_TEST__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(SUB_TEST_BUILDING_DLL) || defined(SUB_TEST_EXPORTS)
    #define SUB_TEST_PUBLIC __declspec(dllexport)
    #define SUB_TEST_LOCAL
  #else  // defined(SUB_TEST_BUILDING_DLL) || defined(SUB_TEST_EXPORTS)
    #define SUB_TEST_PUBLIC __declspec(dllimport)
    #define SUB_TEST_LOCAL
  #endif  // defined(SUB_TEST_BUILDING_DLL) || defined(SUB_TEST_EXPORTS)
#elif defined(__linux__)
  #define SUB_TEST_PUBLIC __attribute__((visibility("default")))
  #define SUB_TEST_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define SUB_TEST_PUBLIC __attribute__((visibility("default")))
  #define SUB_TEST_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // SUB_TEST__VISIBILITY_CONTROL_HPP_
