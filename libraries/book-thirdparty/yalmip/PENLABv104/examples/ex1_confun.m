function [g,userdata] = ex1_confun(x,Y,userdata)
% function values for Example 1

  g = zeros(2,1);
  g(1) = x(1)^2 + x(2)^2 + x(3)^2;
  g(2) = 2*x(1) + 6*x(2) + 4*x(3);
  
