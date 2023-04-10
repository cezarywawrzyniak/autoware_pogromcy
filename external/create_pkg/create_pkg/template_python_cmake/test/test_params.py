#!/usr/bin/env python3

# Copyright YEAR MAINTAINER_NAME
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import pytest
from hello_world.hello_world import HelloWorld


@pytest.mark.parametrize("test_input, expected", [
    (999, 123)
])
def test_default_param(test_input, expected):
    hello_world = HelloWorld()
    assert hello_world.printHello() == expected, "Wrong default value"


@pytest.mark.parametrize("test_input, expected", [
    (456, 456)
])
def test_custom_param(test_input, expected):
    hello_world = HelloWorld()
    hello_world.setParameters(param_name=test_input)
    assert hello_world.printHello() == expected, "Wrong value after parametrization"
