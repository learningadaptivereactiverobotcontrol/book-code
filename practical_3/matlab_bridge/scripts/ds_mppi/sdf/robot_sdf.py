#implementation is based on https://github.com/NVlabs/storm

import torch
from torch import nn
from torch.nn import Sequential as Seq, Linear as Lin, ReLU, ELU, ReLU6, Tanh
#from .network_macros_mod import MLPRegression, scale_to_base, scale_to_net
from network_macros_mod import *
#from .util_file import *
from functools import partial
from functorch import vmap, vjp
from functorch.compile import aot_function, ts_compile

class RobotSdfCollisionNet():
    """This class loads a network to predict the signed distance given a robot joint config."""
    def __init__(self, in_channels, out_channels, skips, layers):

        super().__init__()
        act_fn = ReLU
        self.in_channels = in_channels
        self.out_channels = out_channels
        dropout_ratio = 0
        mlp_layers = layers
        self.model = MLPRegression(self.in_channels, self.out_channels, mlp_layers, skips, act_fn=act_fn, nerf=True)
        # self.m = torch.zeros((500, 1)).to('cuda:0')
        # self.m[:, 0] = 1
        self.order = list(range(self.out_channels))

    def set_link_order(self, order):
        self.order = order

    def load_weights(self, f_name, tensor_args):
        """Loads pretrained network weights if available.

        Args:
            f_name (str): file name, this is relative to weights folder in this repo.
            tensor_args (Dict): device and dtype for pytorch tensors
        """
        try:
            chk = torch.load(f_name, map_location=torch.device('cpu'))
            self.model.load_state_dict(chk["model_state_dict"])
            self.norm_dict = chk["norm"]
            for k in self.norm_dict.keys():
                self.norm_dict[k]['mean'] = self.norm_dict[k]['mean'].to(**tensor_args)
                self.norm_dict[k]['std'] = self.norm_dict[k]['std'].to(**tensor_args)
            print('Weights loaded!')
        except Exception as E:
            print('WARNING: Weights not loaded')
            print(E)
        self.model = self.model.to(**tensor_args)
        self.tensor_args = tensor_args
        self.model.eval()

    def compute_signed_distance(self, q):
        """Compute the signed distance given the joint config.

        Args:
            q (tensor): input batch of joint configs [b, n_joints]

        Returns:
            [tensor]: largest signed distance between any two non-consecutive links of the robot.
        """
        with torch.no_grad():
            q_scale = scale_to_net(q, self.norm_dict, 'x')
            dist = self.model.forward(q_scale)
            dist_scale = scale_to_base(dist, self.norm_dict, 'y')
        return dist_scale[:, self.order].detach()

    def compute_signed_distance_wgrad(self, q, idx = 'all'):
        minidxMask = torch.zeros(q.shape[0])
        if idx == 'all':
            idx = list(range(self.out_channels))
        if self.out_channels == 1:
            with torch.enable_grad():
                q.requires_grad = True
                q.grad = None
                q_scale = scale_to_net(q, self.norm_dict, 'x')
                dist = self.model.forward(q_scale)
                dist_scale = scale_to_base(dist, self.norm_dict, 'y').detach()
                m = torch.zeros((q.shape[0], dist.shape[1])).to(q.device)
                m[:, 0] = 1
                dist.backward(m)
                grads = q.grad.detach()
                # jac = torch.autograd.functional.jacobian(self.model, q_scale)
        else:
            with torch.enable_grad():
                #https://discuss.pytorch.org/t/derivative-of-model-outputs-w-r-t-input-features/95814/2
                q.requires_grad = True
                q.grad = None
                #removed scaling as we don't use it
                dist_scale = self.model.forward(q)
                dist_scale = dist_scale[:, self.order]
                minidxMask = torch.argmin(dist_scale, dim=1)
                grd = torch.zeros((q.shape[0], self.out_channels), device = q.device, dtype = q.dtype) # same shape as preds
                if type(idx) == list:
                    grads = torch.zeros((q.shape[0], q.shape[1], len(idx)))
                    for k, i in enumerate(idx):
                        grd *= 0
                        grd[:, i] = 1  # column of Jacobian to compute
                        dist_scale.backward(gradient=grd, retain_graph=True)
                        grads[:, :, k] = q.grad  # fill in one column of Jacobian
                        q.grad.zero_()  # .backward() accumulates gradients, so reset to zero
                else:
                    grads = torch.zeros((q.shape[0], q.shape[1], 1), device = q.device, dtype = q.dtype)
                    grd[list(range(q.shape[0])), minidxMask] = 1
                    dist_scale.backward(gradient=grd, retain_graph=False)
                    grads[:, :, 0] = q.grad  # fill in one column of Jacobian
                    #q.grad.zero_()  # .backward() accumulates gradients, so reset to zero
                    for param in self.model.parameters():
                        param.grad = None
        return dist_scale.detach(), grads.detach(), minidxMask.detach()

    def allocate_gradients(self, N, tensor_args):
        self.grads = torch.zeros((N, self.in_channels, 1)).to(**tensor_args)
        self.grd = torch.zeros((N, self.out_channels)).to(**tensor_args)
        self.maxInputSize = N

    def dist_grad_closest(self, q):
        n_inputs = min(self.maxInputSize, q.shape[0])
        q = q[:n_inputs]
        self.grd = self.grd * 0
        self.grads = self.grads * 0
        with torch.enable_grad():
            #https://discuss.pytorch.org/t/derivative-of-model-outputs-w-r-t-input-features/95814/2
            q.requires_grad = True
            q.grad = None
            #removed scaling as we don't use it
            dist_scale = self.model.forward(q)
            minidxMask = torch.argmin(dist_scale, dim=1)
            #self.grd = torch.zeros((q.shape[0], self.out_channels), device = q.device, dtype = q.dtype) # same shape as preds
            #self.grads = torch.zeros((q.shape[0], q.shape[1], 1), device = q.device, dtype = q.dtype)
            self.grd[list(range(n_inputs)), minidxMask] = 1
            dist_scale.backward(gradient=self.grd[:n_inputs], retain_graph=False)
            self.grads[:n_inputs, :, 0] = q.grad  # fill in one column of Jacobian
            #q.grad.zero_()  # .backward() accumulates gradients, so reset to zero
            for param in self.model.parameters():
                param.grad = None
        return dist_scale.detach(), self.grads[:n_inputs].detach(), minidxMask.detach()


    def tmp_fcn(self, model, q):
        dist_scale = model.forward(q)
        dist_scale.backward()
        return dist_scale, q.grad

    def compute_signed_distance_wgrad2(self, q):
        grad_map = torch.zeros(7, q.shape[0], 7, device = q.device, dtype = q.dtype)
        dists, vjp_fn = vjp(partial(self.model.forward), q)
        minidxMask = torch.argmin(dists, dim=1)
        grad_map[minidxMask, list(range(q.shape[0])), minidxMask] = 1
        ft_jacobian = (vmap(vjp_fn)(grad_map))[0].sum(0)
        return dists.detach(), ft_jacobian.detach(), minidxMask.detach()

    def functorch_vjp(self, points):
        dists, vjp_fn = vjp(self.model.forward, points)
        minIdx = torch.argmin(dists, dim=1)
        grad_v = torch.zeros(points.shape[0], self.out_channels).to(points.device)
        grad_v[list(range(points.shape[0])), minIdx] = 1
        return dists.detach(), vjp_fn(grad_v)[0].detach(), minIdx.detach()

    def dist_grad_closest_aot(self, q):
        return self.aot_lambda(q)
        # return self.functorch_vjp(q)

    def update_aot_lambda(self):
        self.aot_lambda = aot_function(self.functorch_vjp, fw_compiler=ts_compile, bw_compiler=ts_compile)
        return 0
