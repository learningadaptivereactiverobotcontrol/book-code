#
# MIT License
#
# Copyright (c) 2020-2021 NVIDIA CORPORATION.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.#

import torch
from torch import nn
from torch.nn import Sequential as Seq, Linear as Lin, ReLU, ReLU6, ELU, Dropout, BatchNorm1d as BN, LayerNorm as LN, \
    Tanh
import numpy as np


def xavier(param):
    """ initialize weights with xavier.

    Args:
        param (network params): params to initialize.
    """
    nn.init.xavier_uniform(param)


def he_init(param):
    """initialize weights with he.

    Args:
        param (network params): params to initialize.
    """
    nn.init.kaiming_uniform_(param, nonlinearity='relu')
    nn.init.normal(param)


def weights_init(m):
    """Function to initialize weights of a nn.

    Args:
        m (network params): pass in model.parameters()
    """
    fn = he_init
    if isinstance(m, nn.Conv2d):
        fn(m.weight.data)
        m.bias.data.zero_()
    elif isinstance(m, nn.Conv3d):
        fn(m.weight.data)
        m.bias.data.zero_()
    elif isinstance(m, nn.Linear):
        fn(m.weight.data)
        if (m.bias is not None):
            m.bias.data.zero_()


def MLP(channels, act_fn=ReLU, islast=False):
    """Automatic generation of mlp given some

    Args:
        channels (int): number of channels in input
        dropout_ratio (float, optional): dropout used after every layer. Defaults to 0.0.
        batch_norm (bool, optional): batch norm after every layer. Defaults to False.
        act_fn ([type], optional): activation function after every layer. Defaults to ReLU.
        layer_norm (bool, optional): layer norm after every layer. Defaults to False.
        nerf (bool, optional): use positional encoding (x->[sin(x),cos(x)]). Defaults to True.

    Returns:
        nn sequential layers
    """
    if not islast:
        layers = [Seq(Lin(channels[i - 1], channels[i]), act_fn())
                  for i in range(1, len(channels))]
    else:
        layers = [Seq(Lin(channels[i - 1], channels[i]), act_fn())
                  for i in range(1, len(channels) - 1)]
        layers.append(Seq(Lin(channels[-2], channels[-1])))

    layers = Seq(*layers)

    return layers


class MLPRegression(nn.Module):
    def __init__(self, input_dims=10, output_dims=1, mlp_layers=[128, 128, 128, 128, 128], skips=[2], act_fn=ReLU,
                 nerf=True):
        """Create an instance of mlp nn model

        Args:
            input_dims (int): number of channels
            output_dims (int): output channel size
            mlp_layers (list, optional): perceptrons in each layer. Defaults to [256, 128, 128].
            dropout_ratio (float, optional): dropout after every layer. Defaults to 0.0.
            batch_norm (bool, optional): batch norm after every layer. Defaults to False.
            scale_mlp_units (float, optional): Quick way to scale up and down the number of perceptrons, as this gets multiplied with values in mlp_layers. Defaults to 1.0.
            act_fn ([type], optional): activation function after every layer. Defaults to ELU.
            layer_norm (bool, optional): layer norm after every layer. Defaults to False.
            nerf (bool, optional): use positional encoding (x->[sin(x),cos(x)]). Defaults to False.
        """
        super(MLPRegression, self).__init__()

        mlp_arr = []
        if (nerf):
            input_dims = 3 * input_dims
        if len(skips) > 0:
            mlp_arr.append(mlp_layers[0:skips[0]])
            mlp_arr[0][-1] -= input_dims
            for s in range(1, len(skips)):
                mlp_arr.append(mlp_layers[skips[s - 1]:skips[s]])
                mlp_arr[-1][-1] -= input_dims
            mlp_arr.append(mlp_layers[skips[-1]:])
        else:
            mlp_arr.append(mlp_layers)

        mlp_arr[-1].append(output_dims)

        mlp_arr[0].insert(0, input_dims)
        self.layers = nn.ModuleList()
        for arr in mlp_arr[0:-1]:
            self.layers.append(MLP(arr, act_fn=act_fn, islast=False))
        self.layers.append(MLP(mlp_arr[-1], act_fn=act_fn, islast=True))

        self.nerf = nerf

    def forward(self, x):
        """forward pass on network."""
        if (self.nerf):
            x_nerf = torch.cat((x, torch.sin(x), torch.cos(x)), dim=-1)
        else:
            x_nerf = x
        y = self.layers[0](x_nerf)
        for layer in self.layers[1:]:
            y = layer(torch.cat((y, x_nerf), dim=1))
        return y

    # def forward_backward(self, x):
    #     """forward pass on network."""
    #     x.requires_grad = True
    #     x.grad = None
    #     if (self.nerf):
    #         x_nerf = torch.cat((x, torch.sin(x), torch.cos(x)), dim=-1)
    #     else:
    #         x_nerf = x
    #     y = self.layers[0](x_nerf)
    #     for layer in self.layers[1:]:
    #         y = layer(torch.cat((y, x_nerf), dim=1))
    #     minidxMask = torch.argmin(y, dim=1)
    #     gradMask = torch.zeros((x.shape[0], x.shape[1]-3), device = x.device, dtype = x.dtype) # same shape as preds
    #     gradMask[list(range(x.shape[0])), minidxMask] = 1
    #     y.backward(gradient=gradMask, retain_graph=False)
    #     return y, x.grad


    def reset_parameters(self):
        """Use this function to initialize weights. Doesn't help much for mlp.
        """
        self.apply(weights_init)


def scale_to_base(data, norm_dict, key):
    """Scale the tensor back to the orginal units.  

    Args:
        data (tensor): input tensor to scale
        norm_dict (Dict): normalization dictionary of the form dict={key:{'mean':,'std':}}
        key (str): key of the data

    Returns:
        tensor : output scaled tensor
    """
    scaled_data = torch.mul(data, norm_dict[key]['std']) + norm_dict[key]['mean']
    return scaled_data


def scale_to_net(data, norm_dict, key):
    """Scale the tensor network range

    Args:
        data (tensor): input tensor to scale
        norm_dict (Dict): normalization dictionary of the form dict={key:{'mean':,'std':}}
        key (str): key of the data

    Returns:
        tensor : output scaled tensor
    """

    scaled_data = torch.div(data - norm_dict[key]['mean'], norm_dict[key]['std'])
    scaled_data[scaled_data != scaled_data] = 0.0
    return scaled_data
