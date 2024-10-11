%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2024 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Aude Billard
%   email: aude.billard@epfl.ch
%   website: lasa.epfl.ch
%    
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 3 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This scripts create an ideal linear DS to be used as a reference 
% compared to real data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
clear; clc;
filepath = fileparts(which('generate_theoretical_DS.m'));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-ds-opt')));
cd(filepath);

rotx = @(t) [1 0 0; 0 cos(t) -sin(t) ; 0 sin(t) cos(t)];
roty = @(t) [cos(t) 0 sin(t) ; 0 1 0 ; -sin(t) 0  cos(t)];
rotz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1];

%% Create and simulate DS
% Define DS as a rotation matrix
target = [0.5; 0; 0.3];
theta = 45;
A = -rotx(theta);
b = -A*target;
ds_seds = @(x) A*x + b;

% Simulate DS starting from random points
x0 = rand(3,10)-[1; 0.5; 0.5];
opt_sim = [];
opt_sim.dt    = 0.005;  % Integration timestep
opt_sim.i_max = 10000;  % Maximum number of iterations
opt_sim.tol   = 1e-3;   % Maximum final velocity at the attractor [m/s]
opt_sim.plot  = 1;
disp('Simulation started. It can take a few minutes depending on your termination conditions')
pause(1)
[x_sim, x_dot_sim, t]    = Simulation(x0, [], ds_seds, opt_sim);
% Resize figure to appear on any screen
screensize = get(groot, 'Screensize');
set(gcf,'Position',[0.1  * screensize(3), 0.1  * screensize(4), 0.6 * screensize(3), 0.7 * screensize(4)]);

%% Sub-sample trajecoties and save them
trajectories = [];
nPoints = 300;  
for i=1:size(x_sim, 3)
    trajectories = cat(3, trajectories, [interp1(t, (x_sim(:, :, i))', linspace(t(1), t(end), nPoints)), ...
                                                   interp1(t, x_dot_sim(:, :, i)', linspace(t(1), t(end), nPoints))]');
end

save("theoretical_DS_dataset.mat", "trajectories");
