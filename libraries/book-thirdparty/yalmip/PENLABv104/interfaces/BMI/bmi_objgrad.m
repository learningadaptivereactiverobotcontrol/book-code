function [df, userdata]=bmi_objgrad(x,Y,userdata)
% return gradient of the objective (vector Nx x 1)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  df = userdata.c + userdata.H*x;

