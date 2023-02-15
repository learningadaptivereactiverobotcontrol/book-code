
function []=testmex()
% TESTMEX tests that the behaviour of all the mex files is correct

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  disp('Testing mextrdsdsmat()...');
  testmextrdsdsmat()

  disp('Testing mexsumsparse()...');
  testmexsumsparse()

end

% run all tests on mextrdsdsmat()
function []=testmextrdsdsmat()
  
  onetestmextrdsds('n=0, all empty', [],[],[],[],0);
  onetestmextrdsds('n=1, all ones', 1,1,1,1,2);
  onetestmextrdsds('n=1, all nonzeros', 2,3,-4,5,-120*2);
  onetestmextrdsds('n=1, A=0', 0,3,-4,5,0);
  onetestmextrdsds('n=1, S1=0', 2,0,-4,5,0);
  onetestmextrdsds('n=1, B=0', 2,3,0,5,0);
  onetestmextrdsds('n=1, S2=0', 2,3,-4,0,0);
  %onetestmextrdsds('n=2, all numbers, A diag', diag([2 3]),[-2.1,3.2;3.2,4.4],magic(2)+3,-magic(2)^2); % magic is not symmetric!!!
  onetestmextrdsds('n=2, all numbers, A diag', diag([2 3]),[-2.1,3.2;3.2,4.4],[4,6;6,5],[-13,-9;-9,-16]);
  onetestmextrdsds('n=2, all numbers, A diag, B=[]', diag([2 3]),[-2.1,3.2;3.2,4.4],magic(2)+3,sparse(2,2),0);
  
  % build special examples with very sparse matrices
  n=5;
  A=diag(1:n);
  B=magic(n)+magic(n)';
  S1=sparse(n,n); S1(2,1)=2;
  S1=S1+S1';
  S2=sparse(n,n); S2(4,4)=9;
  onetestmextrdsds('n=5, very sparse', A,S1,B,S2);

  n=5;
  A=diag(1:n);
  B=magic(n)+magic(n)';
  S1=sparse(n,n); S1(2,1)=2;
  S1=S1+S1';
  S2=sparse(n,n); S2(2,2)=9;
  onetestmextrdsds('n=5, very sparse2', A,S1,B,S2);

  n=5;
  A=zeros(n,n);
  A(:)=[1:n^2]-10;
  A=A+A';
  B=magic(n)+magic(n)';
  S1=sparse(n,n); S1(2,1)=2;
  S1=S1+S1';
  S2=sparse(n,n); S2(2,2)=9;
  onetestmextrdsds('n=5, very sparse3', A,S1,B,S2);

  n=5;
  A=zeros(n,n);
  A(:)=[1:n^2]-10;
  A=A+A';
  B=magic(n)+magic(n)';
  S1=sparse(n,n); S1(2,1)=2;
  S1=S1+S1';
  S2=sparse(diag([5:9]));
  onetestmextrdsds('n=5, sparse, S2 spdiag', A,S1,B,S2);


  % random of different size
  for n=[5 5 5 10 10 10 100 100 999 1000]
  %for n=[2 5 5]
    A=rand(n,n);
    A=A+A';
    B=rand(n,n);
    B=B+B';
    S1=sprandsym(n,0.1);
    S2=sprandsym(n,0.05);
    testname=sprintf('n=%i all random',n);
    onetestmextrdsds(testname, A,S1,B,S2);
  end


end

% run one test with mextrdsdsmat()
%   testname - string to print out
%   A,B,S1,S2 - matrices for the test
%   red [optional] - reference result, if not present, computed in Matlab
function [ok]=onetestmextrdsds(testname,A,S1,B,S2,ref)

  ok=false;

  if (~issparse(S1))
    S1=sparse(S1);
  end
  if (~issparse(S2))
    S2=sparse(S2);
  end

  if (nargin==5)
    %ref = trace(A*S1*B*S2);
    ref = trace(A*S1*B*S2) + trace(A*S2*B*S1);
  end
  %result = mextrdsdsmat(A,S1,B,S2);
  result = mextrcolumn(A,S1,B,{S2});

  if (result==ref)
    fprintf(' test %-30s: ok %15.6e\n', testname,result);
    ok=true;
  elseif (abs(result-ref)<1e-10*(abs(result+ref)))
    fprintf(' test %-30s: ok (tol)  %15.6e %15.6e\n', testname,ref,result);
    ok=true;
  else
    fprintf(' test %-30s: FAILED  %15.6e %15.6e\n', testname,ref,result);
  end

end

% run all tests on mexsumsparse()
function [] = testmexsumsparse()

  % some of Si will be empty...
  %onerandtestmexsumsparse('n=1, a few',1,20,0.8);

  onerandtestmexsumsparse('n=5, a few',5,20,0.8);

  onerandtestmexsumsparse('n=30 x1, a few',30,1,0.3);
  onerandtestmexsumsparse('n=30 x2, a few',30,2,0.3);
  onerandtestmexsumsparse('n=30 x50, a few',30,50,0.1);

  onerandtestmexsumsparse('n=401 x50, a few',401,50,0.1);

  %onerandtestmexsumsparse(testname,n,m,density);

end

% run one random test of mexsumsparse
%   n - dimension of matrices
%   m - number of matrices
%   density (0..1) - density of the matrices
function [ok] = onerandtestmexsumsparse(testname,n,m,density)

  ok = false;

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

  % call mex to generate result as dense
  rd = mexsumsparse(w,S,-1);

  err = norm(ref-rd,inf);
  nrm = norm(ref,inf);

  if (err<1e-10*nrm)
    fprintf(' test (dense) %-30s: ok (tol)  %15.6e %15.6e\n', testname,err,nrm);
    ok=true;
  else
    fprintf(' test (dense) %-30s: FAILED  %15.6e %15.6e\n', testname,err,nrm);
  end

  % call mex to generate result as sparse (auto)
  rd = mexsumsparse(w,S);

  err = norm(ref-rd,inf);

  if (err<1e-10*nrm)
    fprintf(' test (auto)  %-30s: ok (tol)  %15.6e %15.6e\n', testname,err,nrm);
    ok=true;
  else
    fprintf(' test (auto)  %-30s: FAILED  %15.6e %15.6e\n', testname,err,nrm);
  end

end
