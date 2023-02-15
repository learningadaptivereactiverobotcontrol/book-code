function [Arf] = packrfp(A)
% PACKRFP assumes a symmetric matrix on input and returns its 'N','L' RFP       
% (rectangular full packed) representation, i.e., a vector of length     
% n*(n+1)/2 with appropriately mapped dense A matrix 

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  [n m] = size(A);
  if (n~=m)
    error('Input matrix needs to be square.')
  end

  if (mod(n,2)==0)
    % n even, q=n/2, lda=n+1
    q=n/2;
    Atemp=zeros(n+1,q);
    Atemp(2:n+1,1:q) = tril(A(1:n,1:q));
    Atemp(1:q,1:q) = Atemp(1:q,1:q) + tril(A(q+1:n,q+1:n))';
  else
    % n odd, q=(n+1)/2, lda=n
    q=(n+1)/2;
    Atemp=zeros(n,q);
    Atemp(1:n,1:q) = tril(A(1:n,1:q));
    Atemp(1:q-1,2:q) = Atemp(1:q-1,2:q) + tril(A(q+1:n,q+1:n))';
  end

  Arf = Atemp(:);

