function [df, userdata]=sdp_objgrad(x,Y,userdata)
% return function value of the objective
% return Nx x 1

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  df = userdata.c;

