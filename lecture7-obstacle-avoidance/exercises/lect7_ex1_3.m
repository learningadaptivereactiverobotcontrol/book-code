%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Aude Billard
%   email:   aude.billard@epfl.ch
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
%  Name of the chapter:  Dynamical system based compliant control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%    Chapter 9 - Obstacle Avoidance: Programming exercise 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;

%% Initialize system
x_limits = [-5, 5];
y_limits = [-5, 5];
nb_gridpoints = 200;

% mesh domain
[x, y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

%% Compute and draw nominal linear DS
A = [-1, 0; ...
     0, -1];  % Linear DS with stable attractor 
x0 = [-1; 1]; % Attractor coordinate
b = -A * x0;  

ds_fun = @(x)(A*x + b);
data = feval(ds_fun, [reshape(x, 1, []); reshape(y, 1, [])]);
x_dot = reshape(data(1,:), nb_gridpoints, nb_gridpoints);
y_dot = reshape(data(2,:), nb_gridpoints, nb_gridpoints);
% 
% % Plot DS
% figure(1);
% hold on; axis equal;
% title('Nominal Linear DS');
% plot_ds(x, y, x_dot, y_dot);
% plot(x0(1), x0(2), 'r*');
% legend('Nominal DS', 'Attractor', 'Location', 'SouthWest');
% xlim(x_limits);
% ylim(y_limits);
% drawnow;

%% ------ Write your code below ------ %%
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

% TODO : Compute a linear DS to move the obstacle

%% Obstacle DS
A_obs % =   %% Linear DS with stable attractor 
x0_obs % = ;
% Attractor coordinate are at Ax+b=0
b_obs % = -A_obs * x0_obs ;  


%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ &
%% ------ Write your code above ------ %%


%% Modulated DS - Simple circular obstacle
figure(2); 
hold on; axis equal;

%% ------ Write your code below ------ %%
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

%% Exercise 2.1.3 
% TODO: 
%   - Set gamma for the case of an ellipse like in 2.1.2
%   - Set and compute reference direction like in 2.1.2
%   - Implement a moving obstacle
%   - DO NOT FORGET to update your obstacle position and velocity 
%           after plotting

% Construct and plot ellipse
object_center = [2; 1];  % Coordinate of center of obstacle
ellipse_axes = [0.5; 2]; % half lengths of ellipse
theta = linspace(0, 2*pi);
x_object = ellipse_axes(1) * cos(theta) + object_center(1);
y_object = ellipse_axes(2) * sin(theta) + object_center(2);

% Set reference point to ensure flow is not stuck
reference_point % = ;

% Set gamma function and its gradient
gamma % = @(x,y) ;
gradient_gamma % = @(x,y) ;

% Compute obstacle velocity for the modulation
object_vel % = ; 

% Initialize variables
x_dot_mod = zeros(nb_gridpoints, nb_gridpoints);
y_dot_mod = zeros(nb_gridpoints, nb_gridpoints);
for i=1:size(x, 1)
    for j=1:size(x, 2)

        % Compute distance function
        distance = gamma(x(i,j), y(i,j));

        % Compute reference direction
        ref_direction % = ;  

        L = [1 - (1 / distance), 0; ...
             0, 1];
        
        % Construct reference and tangent directions
        normal = gradient_gamma(x(i,j), y(i,j));
        normal = normal / vecnorm(normal);
        tangent = [normal(2); -normal(1)];

        E % = ;
        
        % Construct modulation matrix
        M = E * L / E;    
        
        % If we are outside the obstacle:
        if distance > 1
            % Modulate the velocity using the obstacle velocity
            w % = ;
            x_dot_mod(i,j) = w(1);
            y_dot_mod(i,j) = w(2);
        end
    end
end

% Plots and updates your plot 
% NO NEED TO EDIT
% WARNING : must be included in loop to update visualization
title('Modulated DS with moving obstacle');
[h_cont, h_stream] = plot_ds(x, y, x_dot_mod, y_dot_mod);
h_att = plot(x0(1), x0(2), 'r*');
h_obj = plot(x_object, y_object, 'b');
h_att_obs = plot(x0_obs(1), x0_obs(2), 'b*');
h_center = plot(reference_point(1), reference_point(2), 'rd');
legend('Modulated DS', 'Modulated DS attractor', 'Obstacle', ...
        'Obstacle attractor', 'Obstacle reference point', 'Location', 'SouthWest')
xlim(x_limits);
ylim(y_limits);
drawnow;
delete(h_cont); delete(h_stream); delete(h_obj); delete(h_att); delete(h_att_obs); delete(h_center)


% Update obstacle position
object_center % = ;



%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ &
%% ------ Write your code above ------ %%

function [h, h_stream] = plot_ds(x, y, x_dot, y_dot)
[~, h] = contourf(x, y, sqrt(x_dot.^2 + y_dot.^2), 80);
set(h, 'LineColor', 'none');
colormap('summer');
c_bar = colorbar;
c_bar.Label.String = 'Absolute velocity';
h_stream = streamslice(x, y, x_dot,y_dot, 2, 'method', 'cubic');
set(h_stream, 'LineWidth', 1);
set(h_stream, 'color', [0. 0. 0.]);
set(h_stream, 'HandleVisibility', 'off');
end
