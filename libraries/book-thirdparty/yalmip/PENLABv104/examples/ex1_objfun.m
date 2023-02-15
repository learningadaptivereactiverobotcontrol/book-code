function [f,userdata] = ex1_objfun(x,Y,userdata)
% objective function values for Example 1

  f = x(1)^2 + 4*x(2)^2 - x(3)^2 + x(1)*x(2) - 2*x(1)*x(3);
  
