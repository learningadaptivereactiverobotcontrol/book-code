function [df, userdata]=ex1_objgrad(x,Y,userdata)
% Gradients for Example 1, note that they are stored in columns!

  df = [ 2.*x(1) + x(2) - 2*x(3); 8.*x(2) + x(1); -2.*x(3) - 2.*x(1)];
  
