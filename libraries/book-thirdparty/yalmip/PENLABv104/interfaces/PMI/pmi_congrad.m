function [dg, userdata]=pmi_congrad(x,Y,userdata)
% return gradients of all function constraints g(x) (matrix Nx x Ng)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  dg=userdata.B';

