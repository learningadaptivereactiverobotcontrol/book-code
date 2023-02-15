function [Ap] = packmat(A)
% PACKMAT assumes a symmetric matrix on input and returns its 'L' packed 
% representation i.e., a dense vector of length n*(n+1)/2 set up
% by columns of the lower triangle of A

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  [n m] = size(A);
  if (n~=m)
    error('Input matrix needs to be square.')
  end

  Ap = zeros(n*(n+1)/2,1);
  offset=1;
  for j=1:n
    len=n-j;
    Ap(offset:offset+len)=A(j:n,j);
    offset=offset+len+1;
  end

