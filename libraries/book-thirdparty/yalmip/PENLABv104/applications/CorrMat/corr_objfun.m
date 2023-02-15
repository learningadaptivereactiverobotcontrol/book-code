function [f,userdata] = corr_objfun(x,Y,userdata)
% Example 7.1 from the PENLAB paper,
% nearest correlation matrix with the constrained condition number.

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  YH = packmat(x(1).*Y{1}-userdata.H);
  f = YH(:)'*YH(:);

