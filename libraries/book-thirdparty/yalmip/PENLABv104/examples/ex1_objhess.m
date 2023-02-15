function [ddf, userdata] = ex1_objhess(x,Y,userdata)
% Hessians of the objective function, Example 1

  ddf = [2, 1, -2; 1, 8, 0; -2, 0, -2];


