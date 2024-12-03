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

import yaml, sys, os
import torch
import time
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import zmq


"""
  Obstacle bridge for MPPI :
    Takes Rviz topic and publishes to ZMQ for MPPI class to perform joitn space obstacle avoidance
"""


class ObstacleStreamerBridge(Node):
    def __init__(self):
        super().__init__('Obstacle_streamer_bridge')
        self.subscription = self.create_subscription(
            Marker,
            '/visualization_marker',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        ## Set absolute path so ros2 can find it
        self.path_to_ds_mppi = '/home/ros2/ros2_ws/src/matlab_bridge/scripts/ds_mppi/'

        # define tensor parameters (cpu or cuda:0 or mps)
        self.params = {'device': 'cpu', 'dtype': torch.float32}

        # Load initial configuration
        self.config = self.read_yaml(os.path.join(self.path_to_ds_mppi, 'config.yaml'))

        ## Initialize ZeroMQ for MPPI Controller
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        # self.zmq_socket.connect("tcp://localhost:%s" % (self.config["zmq"]["obstacle_port"]))    
        self.zmq_socket.bind("tcp://localhost:%s" % (self.config["zmq"]["obstacle_port"]))    

        # Create obstacles
        self.predefined_obstacles()

        # Define streaming frequency and start timer
        self.freq = self.config["obstacle_streamer"]["frequency"]
        self.timer = self.create_timer(1.0 / self.freq, self.main_loop)

        self.N_ITER = 0
        self.t_0 = time.time()

        ## Params for moving obstacles
        # self.amplitude_array = torch.tensor([[0.0, 0.0, 0.0, 0],
        #                                 [0.0, 0.0, 0.0, 0]])
        # self.period_array = [40, 0.01]

        # delta = obs * 0
        # for i in range(len(self.amplitude_array)):
        #     delta += self.amplitude_array[i] * np.sin(2 * np.pi * t_run / self.period_array[i])

        ## obs+delta
        
        ## Params for keeping track of obstacles
        self.ids_list = []
        self.size_list = []

        ## HACK to always initialize mppi correctly, send far-away and small --> obstacle 0 (MATLAB IDs start at 1)
        self.matlab_obstacles = torch.tensor([[5.0, 5.0, 5.0, 0.001]])
        self.ids_list.append(0)

        self.predef_obs = None

    def listener_callback(self, msg):

        # Convert ROS message to torch tensor for MPPI and keep in memory
        ## obs = torch.tensor([x0, y0, z0, r]) ## all obs are spheres 
        ## for obs in all_obstacles_received:
        ##      all_obs_to_send = torch.vstack((all_obs_to_send, obs))
        
        if msg.ns == 'world' : ## single obstacle

            ## grab obstacle info
            obs = torch.tensor([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.scale.x/2])
            # self.get_logger().info(f"obs ID : {msg.id} ")
            # self.get_logger().info(f"Sending obs : {self.matlab_obstacles}")

            if msg.id not in self.ids_list: ## create new obstacle
                self.ids_list.append(msg.id)
                self.matlab_obstacles = torch.vstack([self.matlab_obstacles, obs])


            elif msg.id in self.ids_list : ## Update existing obstacle
                ## Delete obstacle (if color is 0)
                if msg.color.r == 0.0 and msg.color.g == 0.0 and msg.color.b == 0.0 and msg.color.a == 0.0:
                    self.get_logger().info(f"REMOVING OBSTACLE WITH MATLAB ID OF : {msg.id}")
                    index = self.ids_list.index(msg.id)  # Find the index of the existing obstacle
                    self.matlab_obstacles = torch.cat((self.matlab_obstacles[:index], self.matlab_obstacles[index+1:]), dim=0)
                    del self.ids_list[index]
                
                ## Update existing obstacle
                else: 
                    index = self.ids_list.index(msg.id)  # Find the index of the existing obstacle
                    self.matlab_obstacles[index, : ] = obs
                
        else :## Predefined obstacle made of several spheres
            if msg.color.r == 0.0 and msg.color.g == 0.0 and msg.color.b == 0.0 and msg.color.a == 0.0: ## MATLAB deleting obstacle
                ## REmoving predf obstacle 
                self.get_logger().info(f"REMOVING OBSTACLE WITH MATLAB ID OF : {msg.id}")
                self.predef_obs = None

            elif msg.ns == 'ring' : 
                self.predef_obs = self.ring
            elif msg.ns == 'tshape':
                self.predef_obs = self.tshape
            elif msg.ns == 'wall':
                self.predef_obs = self.wall
            elif msg.ns == 'line':
                self.predef_obs = self.line

       
            # size = self.size_list[index]
            # self.get_logger().info(f"Updating obs : {self.matlab_obstacles} at index {index}, of size {size}")
            # self.matlab_obstacles[index:index+size, : ] = obs
            # self.get_logger().info(f"Updating obs : {self.matlab_obstacles} by changing {obs}")
    

    def predefined_obstacles(self):
        ########################################
        ### I-Shape centered in front of franka
        ########################################
        x_dist = 0.5
        y_width = 0.4
        z_0 = 0.1
        height = 0.75
        r = 0.05
        n_vertical = 20
        n_horizontal = 20

        # # small one
        # z_0 = 0.35
        # height = 0.4
        # y_width = 0.2
        # x_dist = 0.35

        top_left = torch.tensor([x_dist, -y_width, z_0+height, r])
        top_right = torch.tensor([x_dist, y_width, z_0+height, r])
        top_bar = top_left + torch.linspace(0, 1, n_horizontal).reshape(-1, 1) * (top_right - top_left)
        bottom_bar = top_bar - torch.tensor([0, 0, height, 0])

        top_mid = top_left + 0.5 * (top_right - top_left)
        bottom_mid = top_mid - torch.tensor([0, 0, height, 0])
        middle_bar = bottom_mid + torch.linspace(0, 1, n_vertical).reshape(-1, 1) * (top_mid - bottom_mid)
        tshape = torch.vstack((top_bar, middle_bar, bottom_bar))
        ########################################
        ### ring constrained
        ########################################

        center = torch.tensor([.55, 0, 0.6])
        radius = 0.2
        n_ring = 21
        ring = torch.zeros(n_ring, 4)
        ring[:, 0] = center[0]
        ring[:, 1] = center[1] + radius * torch.cos(torch.linspace(0, 2 * np.pi, n_ring))
        ring[:, 2] = center[2] + radius * torch.sin(torch.linspace(0, 2 * np.pi, n_ring))
        ring[:, 3] = 0.03

        ########################################
        ### line/wall
        ########################################
        r = 0.05
        n_pts = 2
        length = max(1, 2*n_pts-2)*r
        z0 = 0.5
        x0 = 0.6
        y0 = 0
        posA = torch.tensor([x0, y0, z0+length, r])
        posB = posA + torch.tensor([length, 0.0, 0.0, 0.0])
        line = posA + torch.linspace(0, 1, n_pts).reshape(-1, 1) * (posB - posA)
        wall = line
        n_down = n_pts
        for sphere in line:
            sphere_down = sphere - torch.tensor([0, 0, length, 0])
            line_down = sphere + torch.linspace(0, 1, n_down).reshape(-1, 1) * (sphere_down - sphere)
            wall = torch.vstack((wall, line_down))
        
        ########################################
        ### Dummy obstacle
        ########################################
        # n_dummy = 1
        # dummy_obs = torch.hstack((torch.zeros(n_dummy, 3) + 10, torch.zeros(n_dummy, 1) + 0.1)).to(**self.params)
        # obs = torch.vstack((obs, dummy_obs)).to(**self.params)

        self.ring = ring
        self.tshape = tshape
        self.wall = wall
        self.line = line

    def read_yaml(self, fname):
        with open(fname) as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
        return config

    def main_loop(self):
        
        ## obs should be :
        ## obs = torch.tensor([x0, y0, z0, r]) ## all obs are spheres 
        ## for obs in all_obstacles_received:
        ##      all_obs_to_send = torch.vstack((all_obs_to_send, obs))

        if self.predef_obs is None:
            self.zmq_socket.send_pyobj(self.matlab_obstacles)
        else:
            all_obstacles = torch.vstack([self.matlab_obstacles, self.predef_obs])
            self.zmq_socket.send_pyobj(all_obstacles)
        # self.get_logger().info('Message sent via ZMQ: "%s"' % str(obs))


        time.sleep(1/self.freq)
        self.N_ITER += 1
        t_run = time.time() - self.t_0

        # if self.N_ITER % self.freq == 0:
            # self.get_logger().info(f"Streaming at {self.freq} Hz for {int(t_run):d} seconds.")
            # self.get_logger().info(f"Sending obs : {self.matlab_obstacles}")


    def destroy(self):
        # Clean up ZMQ context when the node is destroyed
        self.zmq_socket.close()
        self.zmq_context.term()
        super().destroy()


def main(args=None):
    rclpy.init(args=args)

    obstacle_streamer_bridge = ObstacleStreamerBridge()

    try:
        rclpy.spin(obstacle_streamer_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly (optional)
        obstacle_streamer_bridge.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()