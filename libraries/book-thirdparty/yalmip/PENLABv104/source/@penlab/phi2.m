  function [ret] = phi2(obj,t)
%  Penalty function for augmented Lagrangian function.
%  function [ret] = phi2(t)  ... penalty function
%    input can be double scalar/vector/... whatever,
%    return values are of the same type
%    uses parameter R from option settings
%

%  log() is natural logarithm in C as well as in Matlab
%  added possibility to treat t even if the input is a vector

  R=obj.allopts.phi_R;

  ret=t;    % otherwise the result would be a column vector
  if (R < 0)
    ind = t < R;
    ret(ind) = -(1+R)^2*log((1+2*R-t(ind)) / (1+R)) + R + .5*R*R;
    ret(~ind) = t(~ind) + .5*t(~ind).^2;
  else
    ind = t < R;
    ret(ind) = -log(1-t(ind));
    ret(~ind) = ((1 - 2*R)*t(~ind) + .5*t(~ind).^2 - .5*(2*R - 3*R*R)) / (1 - R) / (1 - R) - log(1 - R);
  end

  return;
  

