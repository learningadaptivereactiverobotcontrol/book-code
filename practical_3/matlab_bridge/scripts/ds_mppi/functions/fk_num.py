import torch
from typing import List, Dict
import numpy as np
# from plots import *


@torch.jit.script
def dh_transform(q, d, theta, a, alpha):
    """
    Denavit-Hartenberg transformation matrix.
    """
    # Compute the transformation matrix
    sa = torch.sin(alpha)
    ca = torch.cos(alpha)
    sq = torch.sin(q + theta)
    cq = torch.cos(q + theta)
    # T = torch.tensor([
    #     [cq,        -sq,    0.0,      a],
    #     [sq * ca,   cq*ca,  -sa,    -d*sa],
    #     [sq * sa,   cq*sa,  ca,     d*ca],
    #     [0.0, 0.0, 0.0, 1.0]]).to(q.device, q.dtype)
    t1 = torch.hstack((cq, -sq, torch.tensor(0.0).to(q.device), a))
    t2 = torch.hstack((sq * ca, cq * ca, -sa, -d * sa))
    t3 = torch.hstack((sq * sa, cq * sa, ca, d * ca))
    t4 = torch.tensor((0.0, 0.0, 0.0, 1.0)).to(q.device)
    T = torch.vstack((t1, t2, t3, t4))
    return T


@torch.jit.script
def dh_fk(q: torch.Tensor, dh_params: torch.Tensor):
    """
    Forward kinematics for a robot with Denavit-Hartenberg parameters.
    """
    # Initialize the transformation matrix
    T = [torch.eye(4).to(q.device)]
    # Loop through each joint
    for i in range(len(q)):
        # Extract the parameters for this joint
        d = dh_params[i, 0]
        theta = dh_params[i, 1]
        a = dh_params[i, 2]
        alpha = dh_params[i, 3]
        # Compute the transformation for this joint
        T_prev = T[-1]
        T.append(T_prev @ dh_transform(q[i], d, theta, a, alpha))
    return T


@torch.jit.script
def numeric_fk_model(q: torch.Tensor, dh_params: torch.Tensor, n_pts: int):
    """
    Caclulate positions of points on the robot arm.
    """
    # Compute the transformation matrices
    n_dof = len(q)
    P_arr = dh_fk(q, dh_params)
    links = torch.zeros((n_dof, n_pts, 3)).to(q.device)
    pts_int = torch.zeros((n_dof, n_pts, 3)).to(q.device)
    # Initialize the points array
    # Loop through each joint
    a = dh_params[:, 2]
    for i in range(n_dof):
        p0 = torch.tensor([0, 0, 0]).to(q.device)
        p1 = torch.hstack((a[i + 1], torch.tensor([0.0, 0.0]).to(q.device)))
        lspan = torch.linspace(0.01, 1, n_pts).unsqueeze(1).to(q.device)
        v = torch.tile(p0, (n_pts, 1)) + torch.tile(p1, (n_pts, 1)) * lspan

        R = P_arr[i + 1][:3, :3]
        T = P_arr[i + 1][:3, 3]
        # Compute the position of the point for this link
        pts = (R @ v.transpose(0, 1)).transpose(0, 1) + T
        links[i] = pts
        pts_int[i] = v
    return links, pts_int


@torch.jit.script
def numeric_fk_model_vec(q: torch.Tensor, dh_params: torch.Tensor, n_pts: int):
    """
    Vectorized version of numeric_fk_model
    """
    n_q = q.shape[0]
    n_dof = q.shape[1]
    link_pts = torch.zeros((n_q, n_dof, n_pts, 3)).to(q.device)
    pts_int = torch.zeros((n_q, n_dof, n_pts, 3)).to(q.device)
    for i in range(n_q):
        link_pts[i], pts_int[i] = numeric_fk_model(q[i], dh_params, n_pts)
    return link_pts, pts_int


@torch.jit.script
def dist_to_point(links: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
    # def dist_to_point(links: torch.Tensor, y: torch.Tensor) -> Dict[str, torch.Tensor]:
    """
    Calculate the distance between the robot links and a point in task space.
    """
    res = torch.empty(3).to(y.device)
    dist = torch.norm(links - y[0:3], 2, 2) - y[3]
    mindist_all, minidx_pts = torch.min(dist, 1)
    mindist, idx_link = torch.min(mindist_all, 0)
    idx_pt = minidx_pts[idx_link]
    # res['mindist'] = mindist
    # res['linkidx'] = idx_link
    # res['ptidx'] = idx_pt
    res[0] = mindist
    res[1] = idx_link
    res[2] = idx_pt
    return res


@torch.jit.script
def dist_to_point_vec(links: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
    # calculate distance between each point of each link of each robot to the point y
    dst = torch.norm(links - y[0:3], 2, 3) - y[3]
    # find the minimum distance and the corresponding link and point
    mindist_all, minidx_pts = torch.min(dst, 2)
    mindist, idx_link = torch.min(mindist_all, 1)
    idx_pt = minidx_pts[torch.arange(mindist.shape[0]), idx_link]
    # gather the minimum distance and the corresponding link and point
    res = torch.stack((mindist, idx_link, idx_pt), 1)
    return res


@torch.jit.script
def dist_to_points_vec(links: torch.Tensor, obs: torch.Tensor) -> torch.Tensor:
    """
    Calculate the distance between the robot links and multiple spheres in task space.
    """
    n_traj = links.shape[0]
    n_obs = obs.shape[0]
    n_links = links.shape[1]
    n_pts = links.shape[2]
    res = torch.empty((n_traj, n_obs, 3)).to(links.device)
    for i in range(n_obs):
        res[:, i, :] = dist_to_point_vec(links, obs[i])
        # for j in range(n_links):
        #     res[j, i, :] = dist_to_point(links[j], obs[i])
    return res


@torch.jit.script
def dist_tens(links: torch.Tensor, obs: torch.Tensor) -> torch.Tensor:
    """
    Calculate the distance between the robot links and multiple spheres in task space.
    """
    obs_pos = obs[:, 0:3]
    obs_rad = obs[:, 3]
    # calculate distance between each point of each link of each robot to each point in obs
    pos_dif = links.unsqueeze(2) - obs_pos.unsqueeze(1)
    dst = torch.norm(pos_dif, 2, 4) - obs_rad.unsqueeze(1)
    # find the minimum distance and the corresponding link and point
    mindist_pts, minidx_pts = torch.min(dst, 3)
    mindist_obs, minidx_obs = torch.min(mindist_pts, 2)
    mindist_link, minidx_link = torch.min(mindist_obs, 1)
    idx_obs = minidx_obs[torch.arange(mindist_link.shape[0]), minidx_link]
    idx_pt = minidx_pts[torch.arange(mindist_link.shape[0]), minidx_link, idx_obs]
    # gather the minimum distance and the corresponding link and point
    res = torch.stack((mindist_link, idx_obs, minidx_link, idx_pt), 1)
    return res


def main():
    params = {'device': 'cpu', 'dtype': torch.float32}
    q = torch.tensor([0, 0]).to(**params)
    dh_a = torch.tensor([0, 3, 3])
    dh_alpha = dh_a * 0
    dh_d = dh_a * 0
    dh_theta = dh_a * 0
    dh_params = (torch.vstack((dh_d, dh_theta, dh_a, dh_alpha)).T).to(**params)
    link_pts, pts_int = numeric_fk_model(q, dh_params, 10)
    y = torch.tensor([1, 1, 0]).to(**params)
    res = dist_to_point(link_pts, y)
    print(0)


if __name__ == '__main__':
    main()
