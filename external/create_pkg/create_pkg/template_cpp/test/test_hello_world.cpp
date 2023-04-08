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

#include "gtest/gtest.h"
#include "hello_world/hello_world.hpp"

TEST(TestHelloWorld, TestHello) {
  std::unique_ptr<hello_world::HelloWorld> hello_world_ =
    std::make_unique<hello_world::HelloWorld>();
  auto result = hello_world_->printHello();
  EXPECT_EQ(result, 123);
}
