function R = rotx(phi)
%ROTX  rotate around X by PHI
%
%	R = ROTX(PHI)
%
% See also: ROTY, ROTZ, ROT, POS.

% $ID$
% Copyright (C) 2005, by Brad Kratochvil

R = [1        0         0; ...
     0 cos(phi) -sin(phi); ...
     0 sin(phi)  cos(phi)];

% this just cleans up little floating point errors around 0 
% so that things look nicer in the display
if exist('roundn'),
  R = roundn(R, -15);
end

end