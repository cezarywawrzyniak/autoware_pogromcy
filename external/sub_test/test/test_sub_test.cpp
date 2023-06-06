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

#include "gtest/gtest.h"
#include "sub_test/sub_test.hpp"

TEST(TestSubTest, TestHello) {
  std::unique_ptr<sub_test::SubTest> sub_test_ =
    std::make_unique<sub_test::SubTest>();
  auto result = sub_test_->printHello();
  EXPECT_EQ(result, 123);
}
