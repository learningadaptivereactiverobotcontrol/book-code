function [f,userdata] = ex3_objfun(x,Y,userdata)
% objective function values for Example 3

  f = 100*(x(2) - x(1)^2)^2 + (1 - x(1))^2;
  
