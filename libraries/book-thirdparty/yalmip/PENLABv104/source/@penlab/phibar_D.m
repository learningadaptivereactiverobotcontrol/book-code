function [ret] = phibar_D(obj,t)
% log-penalty barrier function, t can be a scalar/vector/...
%

  ret=t;    % otherwise the result would be a column vector
  ind = find(t<0);
  invind = find(t>=0);
  ret(invind) = -Inf;
  ret(ind) = -1./t(ind);
  return;

