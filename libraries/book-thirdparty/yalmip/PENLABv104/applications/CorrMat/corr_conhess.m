function [ddgk, userdata] = corr_conhess(x,Y,k,userdata)
% Example 7.1 from the PENLAB paper,
% nearest correlation matrix with the constrained condition number.

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  n = length(Y{1});
  nn = n*(n+1)/2;
  idiag = userdata.idiag;
  
  ddgk = zeros((nn+1),(nn+1));
  ddgk(1,1) = 1;
  ddgk(idiag(k)+1,idiag(k)+1) = 1;

