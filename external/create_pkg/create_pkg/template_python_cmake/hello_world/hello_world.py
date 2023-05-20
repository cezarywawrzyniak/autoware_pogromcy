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


class HelloWorld:
    def __init__(self) -> None:
        self._param_name = 123

    def setParameters(self, **kwargs) -> None:
        for arg in kwargs:
            if hasattr(self, "_" + arg):
                setattr(self, "_" + arg, kwargs[arg])

    def printHello(self) -> int:
        print(f"Hello World, {self._param_name}")
        return self._param_name
