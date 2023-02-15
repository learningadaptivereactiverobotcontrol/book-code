function [ddf, userdata] = ncm1a_objhess(x,Y,userdata)
% Hessians of the objective function and constraints

  ddf = diag([2,4,2,4,2]);

  %ddf = diag([2,2,2,2,2]);


