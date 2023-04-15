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

#include "gtest/gtest.h"
#include "save_trajectory/save_trajectory.hpp"

TEST(TestSaveTrajectory, TestHello) {
  std::unique_ptr<save_trajectory::SaveTrajectory> save_trajectory_ =
    std::make_unique<save_trajectory::SaveTrajectory>();
  auto result = save_trajectory_->printHello();
  EXPECT_EQ(result, 123);
}
