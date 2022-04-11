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
%           Dx=M(x, phi_c, Influence)*A*(x-Target)     x(0)=Initial
%   input -----------------------------------------------------------------
%
%       o Target      : (2 x 1), The target Position.  (Optional)
%       o Influence   : (1 x 1), The influence of the modulation function.  
%                               (Optional)
%       o Phi_c       : (1 x 1), The rotation angle of the modulation function.  
%                               (Optional)
%       o A           : (2 x 2), The linear nominal system.   (Optional)
%  How to run -------------------------------------------------------------
%    modulating_dynamical_systems() 
%    modulating_dynamical_systems([0;0])
%    modulating_dynamical_systems([0;0], 10)
%    modulating_dynamical_systems([0;0], 10, 1.57)
%    modulating_dynamical_systems([2;2], 10, 1.57, -4*eye(2))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function modulating_dynamical_systems(Target, Influence, Phi_c, A)
close all; clc;

folder = fileparts(which('modulating_dynamical_systems.m'));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'Local_Modulation')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'tools')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'gp-mds')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'gpml')));

fig1 = figure();
axis equal
title('Feasible Robot Workspace','Interpreter','LaTex')

if ((exist('Target')==0))
    % The target position of the DS
    Target = [0;0];
else
    if all(size(Target)~=[2,1])
        disp('The dimension of the target is not right')
        return;
    end
end

if ((exist('Influence')==0))
    % The influence of the modulation function
    Influence=10;
else
    if all(size(Influence)~=[1,1])
        disp('The dimension of Influence is not right')
        return;
    end
    if Influence<0
        disp('Influence should be positive.')
        return;
    end
end

if ((exist('Phi_c')==0))
    % Compute Random Rotation Angle phi_c
    a = 0; b =3.14;
    Phi_c = a+(b-a)*rand(1,1);
else
    if all(size(Phi_c)~=[1,1])
        disp('The dimension of phi_c is not right')
        return;
    end
    if Phi_c>6.2832
        disp('phi_c should be between 0 and 2\pi')
        return;
    end
    if Phi_c<0
        disp('phi_c should be between 0 and 2\pi')
        return;
    end
end

if ((exist('A')==0))
    % nominal dynamical system
    A=-4*eye(2);
else
    if all(size(A)~=[2,2])
        disp('The dimension of A is not right')
        return;
    end
end

% Axis limits
limits = [-2+Target(1) 2+Target(1) -2+Target(2) 2+Target(2)];

% Plot Attractor
scatter(Target(1),Target(2),50,[0 0 0],'+'); hold on;

%% ------ Write your code below ------
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%

% Modify code here to make the rotation angle Phi_C vary with time.
% Hint : a loop with a varying parameter simulates time.
%
% IMPORTANT : You should also call "simulate_lmds" with a 7th parameter 
% corresponding to the position of modulation location 
% to deactivate the user input in the GUI.
% Example : simulate_lmds(Target, limits,fig1, Influence, Phi_c, A, [1,1]);
% vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

simulate_lmds(Target, limits,fig1, Influence, Phi_c, A);


%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
%% ------ Write your code above ------