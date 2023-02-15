function [penm] = dlyapu(A)
% DLYAPU prepares LMI data for PENLAB
% For a symmetric matrix A, we want to solve the discrete time Lyapunov LMI:
%
% Find a matrix P that minimizes trace(P) and such that
% A'*P*A - P < 0 , P > I
%
% The output is the structure penm, an input for PENLAB
%
% Example of use:
%
% A = [.7 -.2 -.1; .5 .4 0; 0 -.5 .9];
% penm = dlyapu(A);
% prob = penlab(penm); solve(prob); P=prob.Y{1};
%
% See also lyapu, yalmip2bmi

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 02 Dec 2013

  n = size(A,1);
  penm = [];
  penm.probname = 'dlyapu';
  penm.comment = 'discrete-time Lyapunov LMI';

  penm.NY = 1;
  penm.Y{1} = ones(n);
  penm.lbY = 1; %lower bound on eigvals of matrix P

  penm.NALIN = 1;
  penm.ubA = 0; %upper bound on eigvals of A'*P*A - P

  penm.objfun  = @(x,Y,userdata) deal(trace(Y{1}), userdata);
  penm.objgrad = @(x,Y,userdata) deal(packmat(eye(n)), userdata);
  penm.objhess = @(x,Y,userdata) deal([], userdata);

  %transform A'*P*A - P into \sum B_i p_i where p_i are elements of
  %vectorized lower triangle of the variable P and B_i are corresponding
  %matrices formed from A
  %store the vectorized matrices B_i in LMI column-wise into one big matrix
  AAaux = kron(A',A')-eye(n^2);
  indAA = svec2(reshape([1:n*n],n,n));  %indices of lower triangle
  % next we add AA_ji to AA_ij (as P is symmetric)
  indl = tril(reshape([1:n*n],n,n),-1); [dum1,dum2,il]=find(indl(:));
  indu = (triu(reshape([1:n*n],n,n),1))'; [dum1,dum2,iu]=find(indl(:));
  for i=1:length(il)
      AAaux(:,il(i)) = AAaux(:,il(i)) + AAaux(:,iu(i));
  end
  %just the lower triangle
  AA = AAaux(:,indAA);  

  penm.mconfun = @(x,Y,k,userdata) deal(A'*Y{1}*A-Y{1}, userdata);
  penm.mcongrad = @(x,Y,k,i,userdata) deal(sparse(reshape(AA(:,i),n,n)), userdata);

end
