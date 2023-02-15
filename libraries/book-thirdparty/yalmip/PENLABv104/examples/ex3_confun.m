function [g,userdata] = ex3_confun(x,Y,userdata)
% function values for Example 3

  g = zeros(2,1);
  g(1) = x(1)^2 + x(2);
  g(2) = x(1)   + x(2)^2;

