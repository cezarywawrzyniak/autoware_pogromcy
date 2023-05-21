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

#ifndef STEER_THE_CAR__STEER_THE_CAR_HPP_
#define STEER_THE_CAR__STEER_THE_CAR_HPP_

#include <cstdint>

#include "steer_the_car/visibility_control.hpp"


namespace steer_the_car
{

class STEER_THE_CAR_PUBLIC SteerTheCar
{
public:
  SteerTheCar();
  void setParameters(int64_t param_name);
  int64_t printHello() const;

private:
  int64_t param_name_{123};
};

}  // namespace steer_the_car

#endif  // STEER_THE_CAR__STEER_THE_CAR_HPP_
