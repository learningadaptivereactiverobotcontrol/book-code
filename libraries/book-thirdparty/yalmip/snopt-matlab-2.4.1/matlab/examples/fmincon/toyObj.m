function [F,G] = toyObj(x)
% function [F,G] = toyObj(x)
%   Return objective value and gradient

F  = [ 3*x(1) + (x(1) + x(2) + x(3))^2 + 5*x(4) ];

G = [ 2*(x(1)+x(2)+x(3)) + 3;
      2*(x(1)+x(2)+x(3));
      2*(x(1)+x(2)+x(3))
      5 ];
