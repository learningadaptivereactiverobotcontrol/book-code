function [status] = testmexsumsparse(verbose,tol)
% TESTMEXSUMSPARSE tests that the behaviour of mexsumsparse() is correct
%
% RETURN
%   status ...  0 failed, 1 all ok
% INPUT
%   verbose ... 0 print only what failed, 1 print progress,
%               optional, default 1
%   tol ....... tolerance when comparing the results
%               optional, default 1e-8
%
% See also mexsumsparse, penlabtest, penlabstresstest
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
    disp(' testing functionality of mexsumsparse()...')
  end

  % is it possible to call it?
  try
    mexsumsparse(3,{sparse([1,2;2,1])});
  catch
    disp(' Error: mexsumsparse - cannot call')
    status = 0;
    return;
  end

  % some of Si will be empty...
  %status = status && onerandtest(verbose,tol,'n=1, a few',1,20,0.8);

  status = status && onerandtest(verbose,tol,'n=5, a few',5,20,0.8);

  status = status && onerandtest(verbose,tol,'n=30 x1, a few',30,1,0.3);
  status = status && onerandtest(verbose,tol,'n=30 x2, a few',30,2,0.3);
  status = status && onerandtest(verbose,tol,'n=30 x50, a few',30,50,0.1);

  status = status && onerandtest(verbose,tol,'n=401 x50, a few',401,50,0.1);

  %status = status && onerandtest(verbose,tol,testname,n,m,density);

end

function [status] = onerandtest(verbose,tol,testname,n,m,density)
% ONERANDTEST run one randomly generated test on mexsumsparse()
%   n - dimension of matrices
%   m - number of matrices
%   density (0..1) - density of the matrices

  status = 1;

  % generate the problem data
  S = cell(m,1);
  w = rand(m,1);
  for i=1:m
    S{i} = sprandsym(n,density);
  end

  % compute reference result (as dense)
  ref = zeros(n,n);
  for i=1:m
    ref = ref + w(i)*S{i};
  end
  nrm = norm(ref,inf);

  % call mex to generate result as dense
  try
    rd = mexsumsparse(w,S,-1);
    err = norm(ref-rd,inf);
  catch
    err = [];
  end

  if (isempty(err))
    fprintf(' test (dense) %-30s: HARD FAILED\n', testname);
    status = 0;
  elseif (err>tol*nrm)
    fprintf(' test (dense) %-30s: FAILED  %15.6e %15.6e\n', testname,err,nrm);
    status = 0;
  elseif (verbose)
    fprintf(' test (dense) %-30s: ok (tol)  %15.6e %15.6e\n', testname,err,nrm);
  end

  % call mex to generate result as sparse (auto)
  try
    rd = mexsumsparse(w,S);
    err = norm(ref-rd,inf);
  catch
    err = [];
  end

  if (isempty(err))
    fprintf(' test (auto) %-30s: HARD FAILED\n', testname);
    status = 0;
  elseif (err>tol*nrm)
    fprintf(' test (auto)  %-30s: FAILED  %15.6e %15.6e\n', testname,err,nrm);
    status = 0;
  elseif (verbose)
    fprintf(' test (auto)  %-30s: ok (tol)  %15.6e %15.6e\n', testname,err,nrm);
  end

end

