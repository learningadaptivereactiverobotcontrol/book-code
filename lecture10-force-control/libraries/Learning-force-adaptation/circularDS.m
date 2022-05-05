function vd = circularDS(x,xc,rd)

% Compute relative position with respect to the center
xp = x-xc;

% Compute current radius
r = sqrt(xp(1:2)'*xp(1:2));

% Compute angle
theta = atan2(xp(2),xp(1));

% Compute desired radial velocity
rd_dot = (rd-r);

vd = zeros(3,1);

omega = pi;

% Compute desired velocity
vd(1) = (rd_dot*cos(theta))-(omega*sin(theta)*r);
vd(2) = (rd_dot*sin(theta))+(omega*cos(theta)*r);
vd(3) = -xp(2); 
end

