import numpy as np
from scipy.io import loadmat
import torch
import copy
import time


class SEDS:
    def __init__(self, fname, attr = None):
        self.dtype = torch.float32
        seds_data = loadmat(fname)
        self.Mu = torch.tensor(seds_data['Mu']).to(self.dtype)
        self.Sigma = torch.tensor(seds_data['Sigma']).to(self.dtype)
        self.Priors = torch.tensor(seds_data['Priors']).to(self.dtype)
        self.q_goal = torch.tensor(seds_data['xT']).to(self.dtype)
        if attr is not None:
            self.q_goal = attr
        self.dof = int(self.Mu.shape[0]/2)
        self.n_gaussians = self.Sigma.shape[2]
        self.Sigma_inv = torch.zeros([self.dof, self.dof, self.n_gaussians]).to(self.dtype)
        self.det = torch.zeros([self.n_gaussians]).to(self.dtype)
        for i in range(self.n_gaussians):
            self.Sigma_inv[:, :, i] = torch.inverse(self.Sigma[:self.dof, :self.dof, i]).to(self.dtype)
            self.det[i] = torch.abs(torch.det(self.Sigma[:self.dof, :self.dof, i])).to(self.dtype)
        self.seds_thr = 1e-2
        self.lin_thr = 1e-2
    def gaussPDF(self, Data, j):
        nbVar, nbData = Data.shape
        Data = Data.t() - self.Mu[:self.dof, j]
        prob = torch.sum((Data @ torch.inverse(self.Sigma[:self.dof, :self.dof, j])) * Data, dim=1)
        prob = torch.exp(-0.5 * prob) / torch.sqrt(
            (2 * torch.tensor(torch.pi) ** nbVar) * (self.det[j]) + torch.tensor(1e-100))
        return prob.squeeze()

    def GMR(self, x):
        nbData = x.shape[1]
        nbVar = self.Mu.shape[0]
        nbStates = self.Sigma.shape[2]
        Pxi = torch.zeros(nbData, nbStates)
        beta = torch.zeros(nbData, nbStates)
        in_dim = torch.arange(0, self.dof)
        out_dim = torch.arange(self.dof, 2 * self.dof)
        y_tmp = torch.zeros(len(out_dim), nbData, nbStates)

        for i in range(nbStates):
            Pxi[:, i] = self.Priors[i] * self.gaussPDF(x, i)
        beta = Pxi / torch.sum(Pxi, dim=1, keepdim=True)
        beta = beta.nan_to_num()
        beta = torch.clamp(beta, min=1e-8)
        for j in range(nbStates):
            y_tmp[:, :, j] = torch.tile(self.Mu[self.dof:, j].unsqueeze(1), (1, nbData)) + \
                             self.Sigma[self.dof:, :self.dof, j] @ self.Sigma_inv[:self.dof, :self.dof, j] @ \
                             (x - torch.tile(self.Mu[:self.dof, j].unsqueeze(1), (1, nbData)))

        beta_tmp = beta.reshape((1, *beta.shape))
        y_tmp2 = torch.tile(beta_tmp, (self.dof, 1, 1)) * y_tmp
        y = y_tmp2.sum(dim=2)

        return y

    def get_velocity(self, x):
        x_dif = x.transpose(-1, -2)-self.q_goal
        dst = x_dif.norm(p=2, dim=0).unsqueeze(-1)
        far_from_target = (dst > self.lin_thr).squeeze()
        y = self.GMR(x_dif)
        y_norm = y.norm(p=2, dim=0).unsqueeze(-1)
        y_lin = -1*torch.diag(torch.ones(self.dof)) @ x_dif
        y_lin_norm = y_lin.norm(p=2, dim=0).unsqueeze(-1)
        if torch.sum(far_from_target) > 0:
            y[:, far_from_target] = y[:, far_from_target] / y_norm[far_from_target]
            y_lin[:, far_from_target] = y_lin[:, far_from_target] / y_lin_norm[far_from_target]
        weak_seds = (y_norm < self.seds_thr).squeeze()
        if torch.sum(weak_seds & far_from_target) > 0:
            y[:, weak_seds & far_from_target] = y_lin[:, weak_seds & far_from_target]
            print('lin!')
        return y[:self.dof, :].transpose(-2, -1)


if __name__ == '__main__':
    ds = SEDS('../content/seds_left10.mat')
    x0 = torch.tensor([1, 0, 0, 0, 0, 0, 0]).unsqueeze(1)+0.1
    start = time.time()
    x0 = x0.repeat(1, 1000)
    for i in range(100):
        y = ds.get_velocity(x0)
    end = time.time()
    print(end - start)
    print(y)