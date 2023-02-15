  function [ret] = phi2_D2(obj,t)
%  Second derivative of penalty function for augmented Lagrangian function.
%  function [ret] = phi2_D2(t)  ... second derivative of the penalty function
%    input can be double scalar/vector/... whatever,
%    return values are of the same type
%

  R=obj.allopts.phi_R;

  ret=t;
  if (R < 0)
    ind = t < R;
    ret(ind) = (1+R)^2 ./ (1+2*R-t(ind)).^2;
    ret(~ind) = 1.;
  else
    ind = t < R;
    ret(ind) = (1 - t(ind)).^(-2);
    ret(~ind) = (1 - R)^(-2);
  end

  return;


