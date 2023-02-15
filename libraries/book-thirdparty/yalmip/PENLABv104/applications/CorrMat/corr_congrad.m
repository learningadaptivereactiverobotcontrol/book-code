function [dg, userdata] = corr_congrad(x,Y,userdata)
% Example 7.1 from the PENLAB paper,
% nearest correlation matrix with the constrained condition number.
% It returns all gradients of (standard) constraints at once
% as a rectangular matrix (Nx+NYnnz) x Ng.

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  n = length(Y{1});
  nn = n*(n+1)/2;
  idiag = userdata.idiag;
  
  dg = sparse((nn+1),n); % #variables x #constraints
  dg(1,:) = diag(Y{1});
  
  for i=1:n
      dg(idiag(i)+1,i) = x(1);
  end
  
