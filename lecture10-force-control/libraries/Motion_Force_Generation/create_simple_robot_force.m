function theRobot = create_simple_robot(varargin)
%% creating the links

a1 = 0.5;
a2 = 0.7;
%   theta d a alpha
L(1) = Link([ 0     0   a1  0], 'standard');
L(2) = Link([ 0     0   a2  0], 'standard');

% mass
L(1).m = 8;
% center of gravity
L(1).r = [-0.5 0 0];

link_radius = 0.1;
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
L(2).m = 8;
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
name = 'simple robot';
if(nargin>0)
%     name = varargin{1};
    basePosition = varargin{1};
else
    basePosition = zeros(3,1);
end

basePose = [1 0 0 basePosition(1)
            0 1 0 basePosition(2)
            0 0 1 0
            0 0 0 1];
qlim = [-pi pi;
        -pi pi];
theRobot = SerialLink(L, 'name', name, ...
    'comment', 'simple two link robot', 'base',basePose,'qlim',qlim);
theRobot.plotopt = {'noshadow','nojaxes', 'nowrist','noname','linkcolor',[0.91 0.41 0.17], 'ortho','noshading','notiles','jointcolor',0*[1,1,1]};

