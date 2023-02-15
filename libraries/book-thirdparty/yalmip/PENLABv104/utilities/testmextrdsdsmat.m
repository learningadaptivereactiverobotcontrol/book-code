function [status] = testmextrdsdsmat(verbose,tol)
% TESTMEXTRDSDSMAT tests that the behaviour of mextrdsdsmat() is correct
% mextrdsdsmat was the first version which is now superseeded by
% mextrcolumn which computest the vector of traces, not just one
% element. Thus mextrdsdsmat can retire at some point.
%
% RETURN
%   status ...  0 failed, 1 all ok
% INPUT
%   verbose ... 0 print only what failed, 1 print progress,
%               optional, default 1
%   tol ....... tolerance when comparing the results
%               optional, default 1e-8
%
% See also mextrdsdsmat, penlabtest, penlabstresstest
%

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 5 Dec 2013

  if (nargin<2)
    tol = 1e-8;
  end
  if (nargin<1)
    verbose = 1;
  end
  status = 1;

  if (verbose)
    disp(' testing functionality of mextrdsdsmat()...')
  end

  % is it possible to call it?
  try
    mextrdsdsmat(1,sparse(1),1,sparse(1));
  catch
    disp(' Error: mextrdsdsmat - cannot call')
    status = 0;
    return;
  end

  status = status && onematrixtest(verbose,tol,'n=0, all empty', [],[],[],[],0);
  status = status && onematrixtest(verbose,tol,'n=1, all ones', 1,1,1,1,2);
  status = status && onematrixtest(verbose,tol,'n=1, all nonzeros', 2,3,-4,5,-120*2);
  status = status && onematrixtest(verbose,tol,'n=1, A=0', 0,3,-4,5,0);
  status = status && onematrixtest(verbose,tol,'n=1, S1=0', 2,0,-4,5,0);
  status = status && onematrixtest(verbose,tol,'n=1, B=0', 2,3,0,5,0);
  status = status && onematrixtest(verbose,tol,'n=1, S2=0', 2,3,-4,0,0);
  status = status && onematrixtest(verbose,tol,'n=2, all numbers, A diag', diag([2 3]),[-2.1,3.2;3.2,4.4],[4,6;6,5],[-13,-9;-9,-16]);
  % don't forget that magic() is not symmetric
  status = status && onematrixtest(verbose,tol,'n=2, all numbers, A diag, B=[]', diag([2 3]),[-2.1,3.2;3.2,4.4],magic(2)+magic(2)'+3,sparse(2,2),0);
  
  % build special examples with very sparse matrices
  n=5;
  A=diag(1:n);
  B=magic(n)+magic(n)';
  S1=sparse(n,n); S1(2,1)=2;
  S1=S1+S1';
  S2=sparse(n,n); S2(4,4)=9;
  status = status && onematrixtest(verbose,tol,'n=5, very sparse', A,S1,B,S2);

  n=5;
  A=diag(1:n);
  B=magic(n)+magic(n)';
  S1=sparse(n,n); S1(2,1)=2;
  S1=S1+S1';
  S2=sparse(n,n); S2(2,2)=9;
  status = status && onematrixtest(verbose,tol,'n=5, very sparse2', A,S1,B,S2);

  n=5;
  A=zeros(n,n);
  A(:)=[1:n^2]-10;
  A=A+A';
  B=magic(n)+magic(n)';
  S1=sparse(n,n); S1(2,1)=2;
  S1=S1+S1';
  S2=sparse(n,n); S2(2,2)=9;
  status = status && onematrixtest(verbose,tol,'n=5, very sparse3', A,S1,B,S2);

  n=5;
  A=zeros(n,n);
  A(:)=[1:n^2]-10;
  A=A+A';
  B=magic(n)+magic(n)';
  S1=sparse(n,n); S1(2,1)=2;
  S1=S1+S1';
  S2=sparse(diag([5:9]));
  status = status && onematrixtest(verbose,tol,'n=5, sparse, S2 spdiag', A,S1,B,S2);

  % random of different size
  for n=[5 5 5 10 10 10 100 100 101 101]
  %for n=[2 5 5]
    A=rand(n,n);
    A=A+A';
    B=rand(n,n);
    B=B+B';
    S1=sprandsym(n,0.2);
    S2=sprandsym(n,0.1);
    testname=sprintf('n=%i all random',n);
    status = status && onematrixtest(verbose,tol,testname, A,S1,B,S2);
  end

end

function [status] = onematrixtest(verbose,tol,testname,A,S1,B,S2,ref)
% ONEMATRIXTEST - run one test with mextrdsdsmat(), S2 has just one element
%   testname - string to print out
%   A,B,S1,S2 - matrices for the test
%   ref [optional] - reference result, if not present, computed in Matlab

  status = 1;

  if (~issparse(S1))
    S1=sparse(S1);
  end
  if (~issparse(S2))
    S2=sparse(S2);
  end

  if (nargin==7)
    %ref = trace(A*S1*B*S2);
    ref = trace(A*S1*B*S2) + trace(A*S2*B*S1);
  end

  try
    result = mextrdsdsmat(A,S1,B,S2);
    err = abs(result-ref);
  catch
    err = [];
  end

  if (isempty(err))
    fprintf(' test %-30s: HARD FAILED\n', testname);
    status = 0;
  elseif (err>tol*abs(ref))
    fprintf(' test %-30s: FAILED  %15.6e %15.6e\n', testname,err,ref);
    status = 0;
  elseif (verbose)
    fprintf(' test %-30s: ok (tol)  %15.6e %15.6e\n', testname,err,ref);
  end

end

