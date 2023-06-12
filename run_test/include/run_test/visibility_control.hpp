// Copyright 2023 Joanna_Walowska
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

#ifndef RUN_TEST__VISIBILITY_CONTROL_HPP_
#define RUN_TEST__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(RUN_TEST_BUILDING_DLL) || defined(RUN_TEST_EXPORTS)
    #define RUN_TEST_PUBLIC __declspec(dllexport)
    #define RUN_TEST_LOCAL
  #else  // defined(RUN_TEST_BUILDING_DLL) || defined(RUN_TEST_EXPORTS)
    #define RUN_TEST_PUBLIC __declspec(dllimport)
    #define RUN_TEST_LOCAL
  #endif  // defined(RUN_TEST_BUILDING_DLL) || defined(RUN_TEST_EXPORTS)
#elif defined(__linux__)
  #define RUN_TEST_PUBLIC __attribute__((visibility("default")))
  #define RUN_TEST_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define RUN_TEST_PUBLIC __attribute__((visibility("default")))
  #define RUN_TEST_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // RUN_TEST__VISIBILITY_CONTROL_HPP_
