function [ddf, userdata] = ncm2y_objhess(x,Y,userdata)
% Hessians of the objective function and constraints

  [n m] = size(Y{1});
  dim = n*(n+1)/2;
  ddf = 2*speye(dim,dim);


