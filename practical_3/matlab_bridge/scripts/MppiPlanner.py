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
from MPPI import *
from zmq_utils import init_subscriber, init_publisher, zmq_init_recv, zmq_try_recv
from LinDS import LinDS
from robot_sdf import RobotSdfCollisionNet


class MPPIPlanner(Node):
    def __init__(self):
        super().__init__('mppi_planner')

        # Define the path to the ds_mppi
        self.path_to_ds_mppi = '/home/ros2/ros2_ws/src/matlab_bridge/scripts/ds_mppi/'

        # Define tensor parameters (cpu or cuda:0 or mps)
        self.params = {'device': 'cpu', 'dtype': torch.float32}

        # Load configuration
        self.config = self.read_yaml(os.path.join(self.path_to_ds_mppi, 'config.yaml'))

        # Initialize ZMQ
        self.zmq_context = zmq.Context()
        self.socket_send_policy = init_publisher(self.zmq_context, 'localhost', self.config["zmq"]["policy_port"])
        self.socket_receive_state = init_subscriber(self.zmq_context, 'localhost', self.config["zmq"]["state_port"])
        self.socket_receive_obs = init_subscriber(self.zmq_context, 'localhost', self.config["zmq"]["obstacle_port"])

        # Initialize variables
        self.policy_data = None
        self.obs = zmq_init_recv(self.socket_receive_obs)

        # Controller setup
        self.setup_controller()

        # Run the main loop periodically
        self.timer = self.create_timer(1.0 / self.config['planner']['desired_frequency'], self.main_loop)

        # Other parameters
        self.N_ITER = 0
        self.t0 = time.time()

        self.all_kernel_fk = []
        self.upd_mask = []
        self.n_ker_array = []

    def read_yaml(self, fname):
        with open(fname) as file:
            return yaml.load(file, Loader=yaml.FullLoader)

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
        self.N_traj = self.config['planner']['n_trajectories']
        self.dt_H = self.config['planner']['horizon']
        self.dt_sim = self.config['planner']['dt']

        # kernel adding thresholds
        self.dst_thr = self.config['planner']['kernel_adding_collision_thr']       # distance to collision (everything below - adds a kernel)
        self.thr_rbf_add = self.config['planner']['kernel_adding_kernels_thr']   # distance to closest kernel (l2 norm of 7d vector difference)
        self.thr_dot_add = self.config['planner']['kernel_adding_dotproduct_thr']       # dot product between ds and obstacle normal

        # Setup MPPI controller
        n_closest_obs = self.config['collision_model']['closest_spheres']
        self.mppi_step = MPPI(self.q_0, self.q_f, self.dh_params, self.obs, self.dt_sim, self.dt_H, self.N_traj, self.DS_ARRAY, self.dh_a, self.nn_model, n_closest_obs)
        self.mppi_step.Policy.sigma_c_nominal = self.config['planner']['kernel_width']
        self.mppi_step.Policy.alpha_s = self.config['planner']['alpha_sampling_sigma']
        self.mppi_step.Policy.policy_upd_rate = self.config['planner']['policy_update_rate']
        self.mppi_step.Policy.p = self.config['planner']['kernel_p']
        self.mppi_step.dst_thr = self.config['planner']['collision_threshold']       # subtracted from actual distance (added threshsold)
        self.mppi_step.ker_thr = self.config['planner']['kernel_update_threshold']   # used to cr
    

    def main_loop(self):
        self.t_iter = time.time()

        # [ZMQ] Receive state from integrator
        state_dict, state_recv_status = zmq_try_recv(self.mppi_step.q_cur, self.socket_receive_state)
        if state_recv_status:
            # print(f"state rec status ok - state: {mppi.q_cur}")
            self.mppi_step.q_cur = state_dict['q']

            if state_dict['ds_idx'] != self.mppi_step.DS_idx:
                self.mppi_step.switch_DS_idx(state_dict['ds_idx'])
                self.get_logger().info('Resetting policy')
                self.mppi_step.Policy.reset_policy()
                self.all_kernel_fk = []

        # [ZMQ] Receive obstacles
        obstacles_data, obs_recv_status = zmq_try_recv(self.mppi_step.obs, self.socket_receive_obs)
        self.mppi_step.update_obstacles(obstacles_data)

        # Update kernel tangential spaces wrt to new obstacles
        if self.config['planner']['update_kernel_bases']:
            self.mppi_step.update_kernel_normal_bases()
        # Sample random policies
        self.mppi_step.Policy.sample_policy()
        # Propagate modulated DS
        # print(f'Init state: {self.mppi_step.q_cur}')
        with record_function("TAG: general propagation"):
            all_traj, closests_dist_all, kernel_val_all, dotproducts_all = self.mppi_step.propagate()

        with record_function("TAG: cost calculation"):
            # Calculate cost
            cost = self.mppi_step.get_cost() # don't delete, writes to self.cost
            best_idx = torch.argmin(cost)
            self.get_logger().info(f'Best cost: {cost[best_idx]}')
            _, tmp = self.mppi_step.shift_policy_means()
            self.upd_mask.append(tmp)
            self.n_ker_array.append(self.mppi_step.Policy.n_kernels)
        # Check trajectory for new kernel candidates and add policy kernels
        kernel_candidates = self.mppi_step.Policy.check_traj_for_kernels(all_traj, closests_dist_all, dotproducts_all, self.dst_thr - self.mppi_step.dst_thr, self.thr_rbf_add, self.thr_dot_add)

        if len(kernel_candidates) > 0:
            rand_idx = torch.randint(kernel_candidates.shape[0], (1,))[0]
            closest_candidate_norm, closest_idx = torch.norm(kernel_candidates - self.mppi_step.q_cur, 2, -1).min(dim=0)
            if closest_candidate_norm < 1e-1:
                idx_to_add = closest_idx
            else:
                idx_to_add = rand_idx
            candidate = kernel_candidates[idx_to_add]
            # some mess to store the obstacle basis
            idx_i, idx_h = torch.where((all_traj == candidate).all(dim=-1))
            idx_i, idx_h = idx_i[0], idx_h[0]
            # add the kernel finally
            self.mppi_step.Policy.add_kernel(kernel_candidates[idx_to_add], closests_dist_all[idx_i, idx_h], self.mppi_step.norm_basis[idx_i, idx_h].squeeze())
            kernel_fk, _ = numeric_fk_model(kernel_candidates[idx_to_add], self.dh_params, 2)
            self.all_kernel_fk.append(kernel_fk[1:].flatten(0, 1))


        # draw best trajectory
        best_idx = torch.argmin(cost)
        best_traj_fk, _ = numeric_fk_model_vec(self.mppi_step.all_traj[best_idx:best_idx+1].view(-1, 7), self.dh_params, 2)

        # [ZMQ] Send current policy to integrator
        data = {'n_kernels': self.mppi_step.Policy.n_kernels,
                'mu_c': self.mppi_step.Policy.mu_c[0:self.mppi_step.Policy.n_kernels],
                'alpha_c': self.mppi_step.Policy.alpha_c[0:self.mppi_step.Policy.n_kernels],
                'sigma_c': self.mppi_step.Policy.sigma_c[0:self.mppi_step.Policy.n_kernels],
                'norm_basis': self.mppi_step.Policy.kernel_obstacle_bases[0:self.mppi_step.Policy.n_kernels],
                'kernel_fk': self.all_kernel_fk,
                'best_traj_fk': best_traj_fk.view(self.dt_H, -1, 3)[-1].unsqueeze(0)}

        self.socket_send_policy.send_pyobj(data)


        self.N_ITER += 1

        self.t_iter = time.time() - self.t_iter
        self.get_logger().info(f'Iteration:{self.N_ITER:4d}, Time:{self.t_iter:4.2f}, Frequency:{1/self.t_iter:4.2f}, Avg. frequency:{self.N_ITER/(time.time()-self.t0):4.2f}, Kernel count:{self.mppi_step.Policy.n_kernels:4d}')
        #print('Position difference: %4.3f'% (self.mppi_step.q_cur - q_f).norm().cpu())


def main(args=None):
    rclpy.init(args=args)

    mppi_planner = MPPIPlanner()

    try:
        rclpy.spin(mppi_planner)
    except KeyboardInterrupt:
        pass
    finally:
        mppi_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
