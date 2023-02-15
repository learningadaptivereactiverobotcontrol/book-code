function [ddf, userdata] = ex3_objhess(x,Y,userdata)
% Hessians of the objective function, Example 3

  ddf = [1200*x(1)^2-400*x(2)+2, -400*x(1); -400*x(1), 200];

