% This is testing/debugging routine, it shouldn't be used by the user!
% Resets the current point, penalty parameters and Lagrangian multipliers.
% Typically to set a known point from the original Pennon to be able to 
% evaluate the Augmented Lagrangian at the set point & compare the results.
% This work probably only for problems without matrix variables!
% Input: matches (more or less) the elements in the object
%   x ... 
function []=setpointall(obj,xall,uxbox,pxbox,uineq,pineq,ueq,UYbox,PYbox,UA,PA)

  if (length(xall)~=obj.Nx+obj.NYnnz)
    error('ERR @ setpointall: wrong dimension of x.');
  end
  obj.xall=xall;

  % TODO

