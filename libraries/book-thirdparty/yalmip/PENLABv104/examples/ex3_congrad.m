function [dg, userdata]=ex3_congrad(x,Y,userdata)
% Gradients for Example 3, note that they are stored in columns!

  dg = [ 2*x(1), 1; 1, 2*x(2)];
  
