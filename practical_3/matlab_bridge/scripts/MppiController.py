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
from std_msgs.msg import Float64MultiArray
import sys
import os
import torch
import time
import numpy as np
import yaml
import zmq

path_to_ds_mppi = '/home/ros2/ros2_ws/src/matlab_bridge/scripts/ds_mppi/'
sys.path.append(os.path.join(path_to_ds_mppi, 'functions/'))
sys.path.append(os.path.join(path_to_ds_mppi, 'sdf/'))
from MPPI import MPPI
from zmq_utils import init_subscriber, init_publisher, zmq_init_recv, zmq_try_recv
from LinDS import LinDS
from robot_sdf import RobotSdfCollisionNet

## Add AICA representation for sim 
import state_representation as sr
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage
from robot_interface import RobotInterface



class MPPIController(Node):    
    def __init__(self):
        super().__init__('mppi_controller')

        # Define the path to the ds_mppi
        self.path_to_ds_mppi = '/home/ros2/ros2_ws/src/matlab_bridge/scripts/ds_mppi/'

        # Define tensor parameters (cpu or cuda:0 or mps)
        self.params = {'device': 'cpu', 'dtype': torch.float32}

        # Load configuration
        self.config = self.read_yaml(os.path.join(self.path_to_ds_mppi, 'config.yaml'))

        # Initialize ZMQ
        self.zmq_context = zmq.Context()
        self.socket_receive_policy = init_subscriber(self.zmq_context, 'localhost', self.config["zmq"]["policy_port"])
        self.socket_send_state = init_publisher(self.zmq_context, 'localhost', self.config["zmq"]["state_port"])
        self.socket_receive_obs = init_subscriber(self.zmq_context, 'localhost', self.config["zmq"]["obstacle_port"])

        # Initialize variables
        self.policy_data = None
        self.obs = zmq_init_recv(self.socket_receive_obs)

        # Setup communications with simulation
        self.command = CommandMessage()
        self.command.control_type = [ControlType.POSITION.value]

        self.robot_interface = RobotInterface("localhost:1601", "localhost:1602")

        # Wait for zmq connection
        while True:
            self.robot_state = self.robot_interface.get_state()
            # self.get_logger().info(f"Waiting to receive robot_state..")
            if self.robot_state:
                self.command.joint_state = sr.JointState().Zero(self.robot_state.joint_state.get_name(), self.robot_state.joint_state.get_names())            
                break
        
        # Setup ros publiher for robot state for matlab
        self.state_publisher_ = self.create_publisher(Float64MultiArray, '/robot_state', 10)

        # Controller setup
        self.setup_controller()

        # Run the main loop periodically
        # self.timer = self.create_timer(1.0 / self.config['integrator']['desired_frequency'], self.main_loop_no_feedback)
        self.timer = self.create_timer(1.0 / self.config['integrator']['desired_frequency'], self.main_loop_feedback)

        # Other parameters
        self.N_ITER_TOTAL = 0
        self.N_ITER_TRAJ = 0
        self.N_SUCCESS = 0
        self.SLEEP_SUCCESS = 1
        self.t0 = time.time()
        self.t_traj_start = self.t0

    def read_yaml(self, fname):
        with open(fname) as file:
            return yaml.load(file, Loader=yaml.FullLoader)

    def close_sockets(self):
        self.socket_receive_obs.close()
        self.socket_receive_policy.close()
        self.socket_send_state.close()
        self.zmq_context.term()

    def setup_controller(self):
        self.DOF = 7
        # Load nn model
        fname = self.config["collision_model"]["fname"]
        self.nn_model = RobotSdfCollisionNet(in_channels=self.DOF + 3, out_channels=9, layers=[256] * 4, skips=[])
        self.nn_model.load_weights(self.path_to_ds_mppi + '/models/' + fname, self.params)
        self.nn_model.model.to(**self.params)

        # Prepare models: standard (used for AOT implementation), jit, jit+quantization
        self.nn_model.model_jit = torch.jit.script(self.nn_model.model)
        self.nn_model.model_jit = torch.jit.optimize_for_inference(self.nn_model.model_jit)
        self.nn_model.update_aot_lambda()

        # Initial state
        self.q_0 = torch.tensor(self.config['general']['q_0']).to(**self.params)
        self.q_f = torch.tensor(self.config['general']['q_f']).to(**self.params)

        # Robot parameters
        self.dh_a = torch.tensor([0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0])
        self.dh_d = torch.tensor([0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107])
        self.dh_alpha = torch.tensor([0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2, 0])
        self.dh_params = torch.vstack((self.dh_d, self.dh_a * 0, self.dh_a, self.dh_alpha)).T.to(**self.params)

        # Dynamic Systems
        self.DS1 = LinDS(self.q_f)
        self.DS2 = LinDS(self.q_0)

        self.DS_ARRAY = [self.DS1, self.DS2]

        # Integration parameters
        self.N_traj = self.config['integrator']['n_trajectories']
        self.dt_H = self.config['integrator']['horizon']
        self.dt_sim = self.config['integrator']['dt']

        # Setup MPPI controller
        n_closest_obs = self.config['collision_model']['closest_spheres']
        self.mppi_step = MPPI(self.q_0, self.q_f, self.dh_params, self.obs, self.dt_sim, self.dt_H, self.N_traj, self.DS_ARRAY, self.dh_a, self.nn_model, n_closest_obs)
        self.mppi_step.dst_thr = self.config['integrator']['collision_threshold']
        self.mppi_step.Policy.alpha_s *= 0

    def publish_state(self):
        data = np.zeros(7)
        now = self.get_clock().now()
        data[0] = now.seconds_nanoseconds()[0] + 1e-9 * now.seconds_nanoseconds()[1]

        for i in range(3):
            data[i + 1] = self.robot_state.ee_state.get_position()[i]
            data[i + 4] = self.robot_state.ee_state.get_linear_velocity()[i]

        msg = Float64MultiArray()
        msg.data = data.tolist()

        self.state_publisher_.publish(msg)
        # self.get_logger().info(f'Published state: {msg.data}')


    def main_loop_no_feedback(self):
        t_iter_start = time.time()

        # [ZMQ] Receive policy from planner
        self.policy_data, policy_recv_status = zmq_try_recv(self.policy_data, self.socket_receive_policy)
        self.mppi_step.Policy.update_with_data(self.policy_data)

        # [ZMQ] Receive obstacles
        obstacles_data, obs_recv_status = zmq_try_recv(self.mppi_step.obs, self.socket_receive_obs)
        self.mppi_step.update_obstacles(obstacles_data)

        # self.get_logger().info(f"obstacle data : {obstacles_data}")

        # Propagate modulated DS
        self.mppi_step.Policy.sample_policy()
        _, _, _, _ = self.mppi_step.propagate()

        # Update current robot state
        self.mppi_step.q_cur = self.mppi_step.q_cur + self.mppi_step.qdot[0, :] * self.dt_sim
        self.mppi_step.q_cur = torch.clamp(self.mppi_step.q_cur, self.mppi_step.Cost.q_min, self.mppi_step.Cost.q_max)

        # [ZMQ] Send current state to planner
        q_des = self.mppi_step.q_cur
        dq_des = self.mppi_step.qdot[0, :]
        state_dict = {'q': q_des, 'dq': dq_des, 'ds_idx': self.mppi_step.DS_idx}

        self.socket_send_state.send_pyobj(state_dict)

        # Send info to Simulation using robot interface
        self.robot_state = self.robot_interface.get_state()
        if not self.robot_state:
            return
        q_des_np = np.concatenate([q_des.numpy(), np.zeros(2)])
        self.command.joint_state.set_positions(q_des_np)
        self.robot_interface.send_command(self.command)

        self.publish_state()

        self.N_ITER_TOTAL += 1
        self.N_ITER_TRAJ += 1

        t_traj = time.time() - self.t_traj_start

        if (torch.norm(self.mppi_step.q_cur - self.mppi_step.qf) < 0.1) and (self.N_ITER_TRAJ > 100):
            self.get_logger().info('Switching DS!!')
            self.mppi_step.switch_DS_idx(self.N_SUCCESS % 2)
            self.N_ITER_TRAJ = 0
            self.N_SUCCESS += 1
            time.sleep(0.1)
            self.t_traj_start = time.time()

        t_iter = time.time() - t_iter_start
        # self.get_logger().info(f'Iteration: {self.N_ITER_TRAJ:4d} ({self.N_ITER_TOTAL:4d} total), '
        #                        f'Time: {t_iter:4.2f}, Frequency: {1/t_iter:4.2f}, '
        #                        f'Avg. frequency: {self.N_ITER_TOTAL / (time.time() - self.t0 - self.N_SUCCESS * self.SLEEP_SUCCESS):4.2f}, '
        #                        f'Kernel count: {self.mppi_step.Policy.n_kernels:4d}, '
        #                        f'Distance to collision: {self.mppi_step.closest_dist_all[0,0]*100:4.2f}cm, '
        #                        f'Kernel weights: {self.mppi_step.ker_w.numpy()}')
        # self.get_logger().info(f'Policy Alpha: {self.mppi_step.Policy.alpha_c[0:self.mppi_step.Policy.n_kernels].numpy()}')

    def main_loop_feedback(self):
        
        self.t_iter_start = time.time()

        # [ZMQ] Receive robot state
        self.robot_state = self.robot_interface.get_state()

        if self.robot_state:
            self.robot_q = torch.tensor(self.robot_state.joint_state.get_positions()[:7]).to(**self.params)
            if self.robot_q is not None and (self.robot_q - self.mppi_step.q_cur).norm(p=2, dim=-1) > 0.5:
                self.get_logger().info('Resetting state to robot state')
                self.mppi_step.q_cur = self.robot_q

            # [ZMQ] Receive policy from planner
            self.policy_data, policy_recv_status = zmq_try_recv(self.policy_data, self.socket_receive_policy)
            if policy_recv_status:
                # self.get_logger().info(f"Received policy status: {policy_recv_status} - Data: {self.policy_data}")
                self.mppi_step.Policy.update_with_data(self.policy_data)

            # [ZMQ] Receive obstacles
            self.obstacles_data, obs_recv_status = zmq_try_recv(self.mppi_step.obs, self.socket_receive_obs)
            if obs_recv_status:
                self.obstacles_data = self.obstacles_data.clone().detach().to(**self.params) 
                self.mppi_step.update_obstacles(self.obstacles_data)
            
            # self.get_logger().info(f"obstacle data : {self.obstacles_data}")


            # Propagate modulated DS
            self.mppi_step.Policy.sample_policy()  # Sample a new policy using planned means and sigmas
            _, _, _, _ = self.mppi_step.propagate()

            # Update current robot state
            q_des = self.mppi_step.q_cur + self.mppi_step.qdot[0, :] * self.dt_sim
            dq_des = self.mppi_step.qdot[0, :]
            self.mppi_step.q_cur = q_des

            # [ZMQ] Send current state to planner
            state_dict = {'q': q_des, 'dq': dq_des, 'ds_idx': self.mppi_step.DS_idx}
            self.socket_send_state.send_pyobj(state_dict)
            # self.get_logger().info(f"State sent: {state_dict}")

            # Send info to Simulation using robot interface
            q_des_np = np.concatenate([q_des.numpy(), np.zeros(2)])
            self.command.joint_state.set_positions(q_des_np)
            self.robot_interface.send_command(self.command)

            # Send info to matlab using ros 2
            self.publish_state()

            # Update counters
            self.N_ITER_TOTAL += 1
            self.N_ITER_TRAJ += 1
            t_traj = time.time() - self.t_traj_start

            # Check if goal is reached and switch DS if needed
            if torch.norm(self.mppi_step.q_cur - self.mppi_step.qf) < 0.1 and self.N_ITER_TRAJ > 100:
                self.get_logger().info('Switching DS!!')
                self.mppi_step.switch_DS_idx(self.N_SUCCESS % 2)
                status = (
                    f'1: Goal reached in {self.N_ITER_TRAJ:3d} iterations ({t_traj:4.2f} seconds). '
                    f'Obs: {self.obstacles_data.shape[0]}, Top obs: {self.obstacles_data[0]}'
                )
                self.get_logger().info(status)
                self.N_ITER_TRAJ = 0
                self.N_SUCCESS += 1
                time.sleep(0.5)
                self.t_traj_start = time.time()

            self.t_iter = time.time() - self.t_iter_start
            # np.set_printoptions(precision=2, suppress=True)
            # print(f"Iteration: {self.N_ITER_TRAJ}({self.N_ITER_TOTAL} total), Time: {self.t_iter:.2f}s, "
            #     f"Frequency: {1/self.t_iter:.2f}Hz, Avg. Frequency: {self.N_ITER_TOTAL/(time.time()-self.t0-self.N_SUCCESS*0.5):.2f}Hz")


def main(args=None):
    rclpy.init(args=args)

    mppi_controller = MPPIController()

    try:
        rclpy.spin(mppi_controller)
    except KeyboardInterrupt:
        pass
    finally:
        mppi_controller.close_sockets()
        mppi_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
