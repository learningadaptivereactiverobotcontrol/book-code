function [ddf, userdata] = corr_objhess(x,Y,userdata)
% Example 7.1 from the PENLAB paper,
% nearest correlation matrix with the constrained condition number.
% Hessians of the objective function.

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013
  
  YH = packmat(x(1).*Y{1}-userdata.H);
  yy = packmat(Y{1});
  n = length(yy);
  ddf = zeros(n+1,n+1);
  
  ddf(1,1) = 2*sum(yy.^2);
  ddf(1,2:n+1) = 2.*(x(1).*yy+YH);
  ddf(2:n+1,1) = 2.*(x(1).*yy'+YH');
  for i= 1:n
      ddf(i+1,i+1) = 2*x(1)^2;
  end

