%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Sina Mirrazavi
%   email:   sina.mirrazavi@epfl.ch
%   website: lasa.epfl.ch
%
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details
%  Name of the chapter: Modulate dynamical systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example: locally_modulating_dynamical_systems
%           Dx=M(x)*A*(x-Target)     x(0)=Initial
%   input -----------------------------------------------------------------
%       o A           : (2 x 2), The linear nominal system.   (Optional)
%
%  How to run -------------------------------------------------------------
%    locally_modulating_dynamical_systems(-4*eye(2))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function locally_modulating_dynamical_systems(A)
close all; clc;

folder = fileparts(which('locally_modulating_dynamical_systems.m'));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'Learning_modulations')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'tools')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'gp-mds')));

fig1 = figure();
axis equal
title('Feasible Robot Workspace','Interpreter','LaTex')
if ((exist('A')==0))
    A=-4*eye(2);
else
    if all(size(A)~=[2,2])
        disp('The dimension of A is not right')
        return;
    end
end

% Axis limits
Target = [0; 0];
limits = [-2+Target(1) 2+Target(1) -2+Target(2) 2+Target(2)];

% Plot Attractor
scatter(Target(1),Target(2),50,[0 0 0],'+'); hold on;


simulate_lmds(Target, limits,fig1, A);
