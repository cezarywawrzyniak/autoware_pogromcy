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

#ifndef SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP_
#define SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP_

#include <cstdint>

#include "save_trajectory/visibility_control.hpp"


namespace save_trajectory
{

class SAVE_TRAJECTORY_PUBLIC SaveTrajectory
{
public:
  SaveTrajectory();
  void setParameters(int64_t param_name);
  int64_t printHello() const;

private:
  int64_t param_name_{123};
};

}  // namespace save_trajectory

#endif  // SAVE_TRAJECTORY__SAVE_TRAJECTORY_HPP_
