function [df, userdata] = corr_objgrad(x,Y,userdata)
% Example 7.1 from the PENLAB paper,
% nearest correlation matrix with the constrained condition number.
% return gradient of the objective w.r.t. all variables

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  YH=svec2(x(1).*Y{1}-userdata.H);
  
  df(1) = sum(2*svec2(Y{1}).*YH);
  df(2:length(YH)+1) = 2*x(1).*YH;
  
  df = df';

