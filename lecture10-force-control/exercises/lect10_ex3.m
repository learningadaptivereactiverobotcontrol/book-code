%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Walid Amanhoud
%   email:   walid.amanhoud@epfl.ch
%   website: lasa.epfl.ch
%    
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%    Chapter 11: Programming exercise 1 - Part 5 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is an implementation of the method offered in 
% Amanhoud, Walid, Mahdi Khoramshahi, Maxime Bonnesoeur, and Aude Billard.
% "Force adaptation in contact tasks with dynamical systems." 
% In 2020 IEEE International Conference on Robotics and Automation (ICRA), 
% pp. 6841-6847. IEEE, 2020.
% A c-version of the code is available at:
% https://github.com/epfl-lasa/ds_based_contact_tasks/tree/feature/adaptation/src
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; close all; clear all;
folder = fileparts(which('lect10_ex3.m'));
addpath(genpath(fullfile(folder, '..', 'libraries', 'Learning-force-adaptation')));

t = 0; % Time
dt = 0.01; % Time step
tend = 50; % Final time
m = 1; % Mass
x = [0.1;0;0.2]; % Initial position
v =[0;0;0]; % Initial velocity
Fs = [0;0;0]; % Surface force
Fd = 0; % Desired force
xc = [0;0;0]; % Circle position
rd = 0.05; % Circle radius
xs = [0;0;0]; % Point on surface
n = [0;0;-1]; % Normal vector to surface
vt = 0.1; % Target velocity

%% Set the parameters of the RBF function to estimate locally the force modulation
% Set a uniform distribution over a grid

grid_center = xc(1:2); % Center position (2D) of the RBF grid
grid_size = [0.06;0.06]; % Size of the grid (x,y)
nb_gaussians = [10;10]; % Nb gaussians (Nx,Ny)
sigma = grid_size(1)/nb_gaussians(1); % Kernel width

%% ------ Write your code below ------
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%

% TODO: create non uniform RBF grid
centers = createRBFGrid(grid_center, grid_size, nb_gaussians); % Create grid

%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
%% ------ Write your code above ------

weights = zeros(length(centers),1); % RBF weights initialize to zero
deltaF = 0; % Correction force
correction = true; % Activate correction (boolean)

%% Simulation loop
while t(end) < tend   
    % Normal distance
    d = max(x(3,end),0);
    
    % Compute desired velocity
    vd = nominalDS(x(:,end), xc, rd, vt, n, d);
    
    % Compute control force;
    Fc = 100*(vd-v(:,end));
    
    %% ------ Write your code below ------
    %  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%
    % TODO: Implement time-varying surface Force
    % Use t(end) to get current time

    % Compute surface force
    Fs(:,end+1) = surfaceForce(x(:,end),xs);
  
    % Modify the adaptation rate here
    alpha = 10*dt; 
    
    %  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
    %% ------ Write your code above ------

    % Compute desired contact force and force correction;
    Fd(end+1) = 0;
    deltaF(end+1) = deltaF(end);
    if (x(3,end) < 0.01)
        tNow = t(end);

        % Modify the desired force here
        Fd(end) = 10;

        if norm(Fs(:,end))>5 && correction && t(end)>7
            % Compute force error along normal of surface
            Fe = Fd(end)-(-Fs(:,end))'*n;
            % Update weights
            weights = weights + alpha*Fe*grad_rbf_2d(x(:,end),weights,centers,sigma);
            deltaF(end) = rbf_2d(x(:,end),weights,centers,sigma);
        end
    end
    % Add contact force + correction to control force
    Fc = Fc+(Fd(end)+deltaF(end))*n;
    
    % Compute acceleration;
    acc = (Fc+Fs(:,end))/m;
    
    % Integrate to get velocity and position;
    v(:,end+1) = v(:,end)+dt*acc;
    x(:,end+1) = x(:,end)+v(:,end)*dt+acc/2*dt^2; % integration for next point
    
    % Update time
    t(end+1) = t(end)+dt;

end

%% Plot trajectory

figure; hold on;
plot3(x(1,:)',x(2,:)',x(3,:)');
scatter3(centers(:,1), centers(:,2), centers(:,3));
grid on;
xlabel('x [m]','interpreter','latex');
ylabel('y [m]','interpreter','latex');
zlabel('z [m]','interpreter','latex');
legend('Trajectory', 'RBF kernel centers');
view([45, 25])
%% Plot measured, desired, desired corrected forces
figure;
plot(t',Fs(3,:)');
grid on;
hold on;
plot(t',Fd','r');
plot(t',Fd'+deltaF','g');

hold off;
legend({'Measured','Desired','Correction'},'interpreter','latex');
xlabel('t [s]','interpreter','latex');
ylabel('Force [N]','interpreter','latex');
