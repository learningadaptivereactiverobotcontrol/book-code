function [ddf, userdata] = bmi_objhess(x,Y,userdata)
% return Hessian of the objective function (matrix Nx x Nx)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  ddf = userdata.H;


