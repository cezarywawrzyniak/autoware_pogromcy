// Copyright YEAR MAINTAINER_NAME
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

#include "hello_world/hello_world.hpp"

#include <iostream>

namespace hello_world
{

HelloWorld::HelloWorld()
{
}

void HelloWorld::setParameters(int64_t param_name)
{
  param_name_ = param_name;
}

int64_t HelloWorld::printHello() const
{
  std::cout << "Hello World, " << param_name_ << std::endl;
  return param_name_;
}

}  // namespace hello_world
