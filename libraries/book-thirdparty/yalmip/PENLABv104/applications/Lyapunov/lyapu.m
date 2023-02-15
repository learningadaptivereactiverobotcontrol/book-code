function [penm] = lyapu(A)
% LYAPU prepares LMI data for PENLAB
% For a symmetric matrix A, we want to solve the continuous time Lyapunov LMI:
%
% Find a matrix P that minimizes trace(P) and such that
% A'*P + P*A < 0 , P > I
%
% The output is the structure penm, an input for PENLAB
%
% Example of use:
%
% A=[0 1 0;0 0 1;-1 -2 -3];
% penm = lyapu(A);
% prob = penlab(penm); solve(prob); P=prob.Y{1};
%
% See also dlyapu, yalmip2bmi

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 02 Dec 2013

  n = size(A,1);
  penm = [];
  penm.probname = 'lyapu';
  penm.comment = 'continuous-time Lyapunov LMI';

  penm.NY = 1;
  penm.Y{1} = ones(n);
  penm.lbY = 1; %lower bound on eigvals of matrix P

  penm.NALIN = 1;
  penm.ubA = 0; %upper bound on eigvals of A'*P + P*A

  penm.objfun  = @(x,Y,userdata) deal(trace(Y{1}), userdata);
  penm.objgrad = @(x,Y,userdata) deal(packmat(eye(n)), userdata);
  penm.objhess = @(x,Y,userdata) deal([], userdata);

  %transform A'*P + P*A into \sum B_i p_i where p_i are elements of
  %vectorized lower triangle of the variable P and B_i are corresponding
  %matrices formed from A
  %store the vectorized matrices B_i in LMI column-wise into one big matrix
  AAaux = kron(A',eye(n)) + kron(eye(n),A');
  indAA = svec2(reshape([1:n*n],n,n));  %indices of lower triangle
  % next we add AA_ji to AA_ij (as P is symmetric)
  indl = tril(reshape([1:n*n],n,n),-1); [dum1,dum2,il]=find(indl(:));
  indu = (triu(reshape([1:n*n],n,n),1))'; [dum1,dum2,iu]=find(indl(:));
  for i=1:length(il)
      AAaux(:,il(i)) = AAaux(:,il(i)) + AAaux(:,iu(i));
  end
  %just the lower triangle
  AA = AAaux(:,indAA);
  
  penm.mconfun = @(x,Y,k,userdata) deal(A'*Y{1} + Y{1}*A, userdata);
  penm.mcongrad = @(x,Y,k,i,userdata) deal(sparse(reshape(AA(:,i),n,n)), userdata);
end
