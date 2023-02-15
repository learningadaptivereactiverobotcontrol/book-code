function [dg, userdata]=ncm2y_congrad(x,Y,userdata)
% returns all constraints at once
% vector Ng x Nx --> 1xNYnnz TODO no ... transpose! (Nx+NYnnz) x Ng
% this needs to be improved to be automatic from size of Y
% Y is organized as lower triangle column oriented

  %dg=[1;0;0;0;0;0; 1;0;0;0;0; 1;0;0;0; 1;0;0; 1;0; 1];

  % only the elements belonging to the diagonal of Y have derivative
  % find their indices

  % this is not the best way but for now ... :-(
  [n m] = size(Y{1});
  dim = n*(n+1)/2;
  M = tril(ones(n)+eye(n));  % my variables are nonzero and these of diag are =2
  idx_allvar = find(M);
  idx_diagvar = find(M(idx_allvar)==2);

  dg = sparse(idx_diagvar, [1:n], ones(n,1), dim,n);

