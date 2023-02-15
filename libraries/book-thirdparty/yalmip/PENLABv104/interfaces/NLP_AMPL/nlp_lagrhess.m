function [ddL, userdata] = nlp_lagrhess(x,Y,v,userdata)
% Hessian of the lagrangian
%   H=nabla^2 f + sum v_i nabla^2 g_i + sum v_(i+..) nabla^2 h_i
% v should be of size N_CONSTR-N_BOUNDS, e.i., in fact 2 in this example
%
% expect that the point (x) didn't change since last grad/value call...

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  ddL = amplf(v);

