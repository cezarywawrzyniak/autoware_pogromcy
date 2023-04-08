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

import rclpy
from rclpy.node import Node
try:
    from hello_world.hello_world import HelloWorld
except ImportError:
    from hello_world import HelloWorld


class HelloWorldNode(Node):
    def __init__(self):
        super().__init__("hello_world_node")
        self.hello_world = HelloWorld()
        param_name = self.declare_parameter('param_name', 456).value
        self.hello_world.setParameters(param_name=param_name)
        self.foo()

    def foo(self):
        self.hello_world.printHello()


def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
