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

#ifndef GAMEPAD_SUPPORT__GAMEPAD_SUPPORT_HPP_
#define GAMEPAD_SUPPORT__GAMEPAD_SUPPORT_HPP_

#include <cstdint>

#include "gamepad_support/visibility_control.hpp"


namespace gamepad_support
{

class GAMEPAD_SUPPORT_PUBLIC GamepadSupport
{
public:
  GamepadSupport();
  void setParameters(int64_t param_name);

private:
  int64_t param_name_{123};
};

}  // namespace gamepad_support

#endif  // GAMEPAD_SUPPORT__GAMEPAD_SUPPORT_HPP_
