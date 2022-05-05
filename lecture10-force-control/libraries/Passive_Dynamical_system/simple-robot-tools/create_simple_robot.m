
function theRobot=create_simple_robot(varargin)
%% creating the links

a1 = 0.8;
a2 = 0.6;
% a3 = 0.6;

%   theta d a alpha
L(1) = Link([ 0     0   a1  0], 'standard');
L(2) = Link([ 0     0   a2  0], 'standard');
% L(3) = Link([ 0     0   a3  0], 'standard');

% mass
L(1).m = 1; % 1
% center of gravity
L(1).r = [-0.5 0 0];

link_radius = 0.1; % 0.05
link_length = 0.5;
Ix = 1/12 * L(1).m*(3*link_radius.^2+link_length.^2);
Iy = Ix;
Iz = L(1).m*link_radius.^2/2;
L(1).I = diag([Ix,Iy,Iz]);
% gear ratio
L(1).G = 0;
% motor inertia
L(1).Jm = 0;
% viscous friction
L(1).B = 0;

% mass
L(2).m = 1;
% center of gravity
L(2).r = [-0.5 0 0];
%intertia matrix (around cog)
L(2).I =diag([Ix,Iy,Iz]);
% gear ration
L(2).G = 0;
% motor inertia
L(2).Jm = 0;
% viscous friction
L(2).B = 0;

% % mass
% L(3).m = 1;
% % center of gravity
% L(3).r = [-0.5 0 0];
% %intertia matrix (around cog)
% L(3).I =diag([Ix,Iy,Iz]);
% % gear ration
% L(3).G = 0;
% % motor inertia
% L(3).Jm = 0;
% % viscous friction
% L(3).B = 0;

name = 'simple robot';
if(nargin>0)
    name = varargin{1};
end

theRobot = SerialLink(L, 'name', name, ...
    'comment', 'simple three link robot');
H_base = eye(4);
H_base(1:3,4) = [-0.9 1 0 ]';
% H_base(1:3,1:3) = rotx(pi);
theRobot.base = H_base;
theRobot.plotopt = {'noshadow','nojaxes', 'nowrist','noname','linkcolor',0.8*[1,1,1], 'ortho','noshading','notiles','jointcolor',0.7*[1,1,1]};

