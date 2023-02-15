function [ddL, userdata] = ex3_lagrhess(x,Y,v,userdata)
% Hessian of the lagrangian, Example 3
%   H=nabla^2 f + sum v_i nabla^2 g_i
% v should be of size NgNLN, thus in this example NgNLN=2

  hess_f = [1200*x(1)^2-400*x(2)+2, -400*x(1); -400*x(1), 200];
  hess_g1 = [2, 0; 0, 0];
  hess_g2 = [0, 0; 0, 2];
  ddL = hess_f + v(1)*hess_g1 + v(2)*hess_g2;

