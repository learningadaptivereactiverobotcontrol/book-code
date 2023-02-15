function [ddL, userdata] = ex1_lagrhess(x,Y,v,userdata)
% Hessian of the lagrangian, Example 1
%   H=nabla^2 f + sum v_i nabla^2 g_i
% v should be of size NgNLN, e.i., in this example NgNLN=1

  hess_f = [2, 1, -2; 1, 8, 0; -2, 0, -2];
  hess_g = 2.*eye(3,3);

  ddL = hess_f + v(1)*hess_g;

