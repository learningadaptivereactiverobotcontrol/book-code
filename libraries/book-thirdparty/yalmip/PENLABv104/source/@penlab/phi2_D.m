  function [ret] = phi2_D(obj,t)
%  Derivative od penalty function for augmented Lagrangian function.
%  function [ret] = phi2_D(t)  ... first derivative of the penalty function
%    input can be double scalar/vector/... whatever,
%    return values are of the same type
%

  R=obj.allopts.phi_R;

  ret=t;
  if (R < 0)
    ind = t < R;
    ret(ind) = (1+R)^2 ./ (1+2*R-t(ind));
    ret(~ind) = 1. + t(~ind);
  else
    ind = t < R;
    ret(ind) = 1 ./ (1-t(ind));
    ret(~ind) = (1 - 2*R + t(~ind)) / (1 - R) / (1 - R);
  end

  return;


