function [df, userdata]=ex3_objgrad(x,Y,userdata)
% Gradients for Example 3, note that they are stored in columns!

  df = [-400*x(1)*(x(2)-x(1)^2) - 2*(1-x(1)); 200*(x(2)-x(1)^2)];
  
