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
import zmq
import json
from nav_msgs.msg import Path

"""
  Path display bridge for simulation:
    Takes both Rviz topic (planned and executed) and publishes to ZMQ for Pybullet sim 
    
"""

class PathZMQBridge(Node):

    def __init__(self):
        super().__init__('path_zmq_bridge')
        self.sub_planned_traj = self.create_subscription(
            Path,
            '/ds_trajectory',
            self.listener_callback_ds,
            10)
        self.sub_planned_traj  # prevent unused variable warning

        self.sub_executed_traj = self.create_subscription(
            Path,
            '/robot_trajectory',
            self.listener_callback_robot,
            10)
        self.sub_executed_traj  # prevent unused variable warning

        self.sub_demonstration = self.create_subscription(
            Path,
            '/demo_trajectory',
            self.listener_callback_demo,
            10)
        self.sub_demonstration  # prevent unused variable warning

        # Initialize ZeroMQ
        self.zmq_context = zmq.Context()
        self.zmq_socket_ds = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket_ds.connect("tcp://0.0.0.0:3456")

        self.zmq_socket_robot = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket_robot.connect("tcp://0.0.0.0:6543")

        self.zmq_socket_demo = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket_demo.connect("tcp://0.0.0.0:5634")

    def convert_msg_to_dict(self, msg: Path):
        # Convert ROS message to dictionary
        msg_dict = {
            'poses': [],
            'frame_id' : msg.header.frame_id
        }

        # Iterate through the poses in the Path message
        for pose in msg.poses:
            pose_dict = {
                'pose': {
                    'position': {
                        'x': pose.pose.position.x,
                        'y': pose.pose.position.y,
                        'z': pose.pose.position.z,
                    },
                }
            }
            msg_dict['poses'].append(pose_dict)

        return msg_dict

    def listener_callback_ds(self, msg):
        # Convert to dictionary
        msg_dict = self.convert_msg_to_dict(msg)

        # Add color for pybullet sim
        msg_dict['color'] = [1, 0, 0] ## red

        # Serialize dictionary to JSON and send via ZMQ
        self.zmq_socket_ds.send_json(msg_dict)

    def listener_callback_robot(self, msg):
        # Convert to dictionary
        msg_dict = self.convert_msg_to_dict(msg)

        # Add color for pybullet sim
        msg_dict['color'] = [0, 1, 0] ## green

        # Serialize dictionary to JSON and send via ZMQ
        self.zmq_socket_robot.send_json(msg_dict)

    def listener_callback_demo(self, msg):
        # Convert to dictionary
        msg_dict = self.convert_msg_to_dict(msg)

        # Add color for pybullet sim
        msg_dict['color'] = [0, 0, 1] ## blue

        # Serialize dictionary to JSON and send via ZMQ
        self.zmq_socket_demo.send_json(msg_dict)

def main(args=None):
    rclpy.init(args=args)
    node = PathZMQBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
