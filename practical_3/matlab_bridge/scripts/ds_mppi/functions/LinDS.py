import numpy as np
import torch
import copy
import time

class LinDS:
    def __init__(self, q_goal):
        # self.q_goal = torch.tensor(q_goal)
        self.tensor_args = {'device': q_goal.device, 'dtype': q_goal.dtype}
        
        self.q_goal = q_goal.clone().detach()
        self.lin_thr = 0.015
        self.dof = q_goal.shape[0]
    def get_velocity(self, x):
        x_dif = x-self.q_goal
        dst = x_dif.norm(p=2, dim=-1)
        y_lin = x_dif @ torch.diag(-1*torch.ones(self.dof).to(**self.tensor_args))
        y_lin_norm = y_lin.norm(p=2, dim=-1).unsqueeze(-1)
        far_from_target = (dst > self.lin_thr)
        if torch.sum(far_from_target) > 0:
            #y_lin[:, far_from_target] = y_lin[:, far_from_target] / y_lin_norm[far_from_target]
            y_lin[far_from_target] = y_lin[far_from_target] / y_lin_norm[far_from_target]

        return y_lin
