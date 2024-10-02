# Copyright 2016 Open Source Robotics Foundation, Inc.
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
from geometry_msgs.msg import Twist
import sys

cmds = ['turn_right', 'turn_left', 'move_forward', 'move_backward']

class MinimalPublisher(Node):

    def __init__(self, cmd):
        super().__init__('text_to_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.execute_command(cmd)

    def execute_command(self, cmd):
        if cmd in cmds:
            msg = Twist()
            if cmd == 'move_forward':
                msg.linear.x = 1.5
                self.get_logger().info('Moving forward')
            elif cmd == 'move_backward':
                msg.linear.x = -1.5
                self.get_logger().info('Moving backward')
            elif cmd == 'turn_right':
                msg.angular.z = -1.5
                self.get_logger().info('Turning right')
            elif cmd == 'turn_left':
                msg.angular.z = 1.5
                self.get_logger().info('Turning left')

            self.publisher_.publish(msg)
        else:
            self.get_logger().info('Wrong command!')

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python3 text_to_cmd_vel.py <command>")
        return

    cmd = sys.argv[1]

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher(cmd)

    # Spin for a short time to allow the publisher to send the message
    rclpy.spin_once(minimal_publisher, timeout_sec=0.1)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
