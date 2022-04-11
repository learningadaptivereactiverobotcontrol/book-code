%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for Appendix A of:                                      %
% "Robots that can learn and adapt" by Billard, Mirrazavi and Figueroa.   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Dominic Reber                                                  %
% email:   dominic.reber@epfl.ch                                          %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%% Step 0: Add library %%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('appA_ex1.m'));
cd(filepath);

%% %%%%%%%%%%% Step 1: Define grid %%%%%%%%%%%%%%%%%%%%%%%%%%%
x_limits = [-3, 3];
y_limits = [-3, 3];
nb_gridpoints = 50;

% mesh domain
[x, y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

%% %%%%%%%%%%% Generate a DS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ------ Write your code below ------
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

% optional: define DS as anonymous function
% ds_fun = @(x)(...)

x_dot % = ... % calculate velocity in x direction at each gridpoint
y_dot % = ... % calculate velocity in y direction at each gridpoint

% Calculate absolute velocity at each gridpoint
abs_vel % = ...

%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ &
%% ------ Write your code above ------

%% %%%%%%%%%%% Calculate path integral %%%%%%%%%%%%%%%%%%%%%%%

dt = 0.05;
iter = 0;
max_iter = 1000;

%% ------ Write your code below ------
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

initial_position % = ... % TODO: sample random position (2x1 array) from within mesh. Hint: use 'unifrnd'
path_integral = initial_position; % path integral is intially a 2x1 array that will store the path and grow to a 2xN array.
% TODO: implement breaking conditions for while loop
while true
    % TODO: integrate DS to get next position of the path integral
    path_integral(:,end+1) = path_integral(:,end) % + ...
    iter % = ...
end

%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ &
%% ------ Write your code above ------

%% %%%%%%%%%%%% Plot Resulting DS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Use the function plot_ds describe below
% plot_ds(...);


function plot_ds(x, y, x_dot, y_dot, path, x0, titleName, x_target)

%   PLOT_DS  Plot a dynamical system on a grid.
%   PLOT_DS(X, Y, X_DOT, Y_DOT, PATH, X0, TITLENAME, X_TARGET) where the 
%   arrays X,Y define the coordinates for X_DOT,Y_DOT and are monotonic and
%   2-D plaid (as if produced by MESHGRID), plots a dynamical system with
%   attractor(s) X0 given as 2xN vector data and name TITLENAME.
%
%   The optional variable X_TARGET given as 2xN vector data can be used to
%   plot additional points of interest (e.g. local modulation points).
    
    hold on;
    
    [~, h] = contourf(x, y, sqrt(x_dot.^2 + y_dot.^2), 80);
    set(h, 'LineColor', 'none');
    colormap('summer');
    c_bar = colorbar;
    c_bar.Label.String = 'Absolute velocity';
    c_bar.Label.Interpreter = 'Latex';
    c_bar.FontSize = 15;
    c_bar.Label.FontSize = 20;
    
    % Plot velocity stream
    h_stream = streamslice(x, y, x_dot, y_dot, 2, 'method', 'cubic');
    set(h_stream, 'LineWidth', 1);
    set(h_stream, 'color', [0. 0. 0.]);
    set(h_stream, 'HandleVisibility', 'off');
    axis equal;

    scatter(x0(1, :), x0(2, :), 100, 'r*', 'LineWidth', 2);

    plot(path(1,:), path(2,:), 'r', 'LineWidth', 3);

    if exist('x_target', 'var')
        scatter(x_target(1, :), x_target(2, :), 100, 'bd', 'LineWidth', 2);
    end

    box on;
    ax = gca;
    ax.XAxis.FontSize = 15;
    ax.YAxis.FontSize = 15;
    xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize', 20);
    ylabel('$x_2$', 'Interpreter', 'LaTex', 'FontSize', 20);
    title(titleName, 'Interpreter', 'LaTex', 'FontSize', 20);
end
