#!/usr/bin/env python3

#############################################################################
##  Copyright (C) 2024 Learning Algorithms and Systems Laboratory, EPFL,
##    Switzerland
##   Author: Aude Billard
##   email: aude.billard@epfl.ch
##   website: lasa.epfl.ch
##    
##   Permission is granted to copy, distribute, and/or modify this program
##   under the terms of the GNU General Public License, version 3 or any
##  later version published by the Free Software Foundation.
##
##   This program is distributed in the hope that it will be useful, but
##   WITHOUT ANY WARRANTY; without even the implied warranty of
##   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
##   Public License for more details
#############################################################################

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import zmq
import json

"""
  Obstacle bridge for simulation:
    Takes Rviz topic and publishes to ZMQ for Pybullet sim 
"""

class ObstacleZMQBridge(Node):

    def __init__(self):
        super().__init__('obstacle_zmq_bridge')
        self.subscription = self.create_subscription(
            Marker,
            '/visualization_marker',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize ZeroMQ
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.bind("tcp://0.0.0.0:4567")

    def listener_callback(self, msg):
        # Convert ROS message to dictionary
        msg_dict = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id,
            },
            'ns': msg.ns,
            'id': msg.id,
            'type': msg.type,
            'action': msg.action,
            'pose': {
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z,
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w,
                },
            },
            'scale': {
                'x': msg.scale.x,
                'y': msg.scale.y,
                'z': msg.scale.z,
            },
            'color': {
                'r': msg.color.r,
                'g': msg.color.g,
                'b': msg.color.b,
                'a': msg.color.a,
            },
            'lifetime': {
                'sec': msg.lifetime.sec,
                'nanosec': msg.lifetime.nanosec,
            },
            'frame_locked': msg.frame_locked,
            # Add other fields as needed
        }

        # Serialize dictionary to JSON and send via ZMQ
        self.zmq_socket.send_json(msg_dict)
        # self.get_logger().info('Message sent via ZMQ: "%s"' % str(msg_dict))

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleZMQBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
