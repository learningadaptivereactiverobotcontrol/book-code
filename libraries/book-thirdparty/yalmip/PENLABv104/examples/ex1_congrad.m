function [dg, userdata]=ex1_congrad(x,Y,userdata)
% Gradients for Example 1, note that they are stored in columns!

  dg = [ [ 2*x(1); 2*x(2); 2*x(3)] , [2; 6; 4]];
  
