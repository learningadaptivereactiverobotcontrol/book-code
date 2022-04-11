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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialize system
clear; close all; clc;

x_limits = [-5, 5];
y_limits = [-5, 5];
nb_gridpoints = 30;

% mesh domain
[X, Y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

x_dot = zeros(nb_gridpoints);
y_dot = zeros(nb_gridpoints);


%% Compute and Draw Nominal Linear DS
A = [-1, 0; 
     0, -1];  %% Linear DS with stable attractor 
x01 = [0; 0];  
b1 = -A*x01; 
% Attractor coordinate are at Ax+b=0 

% Compute the DS
for i=1:nb_gridpoints
    for j=1:nb_gridpoints

        w = A * [X(i,j); Y(i,j)] + b1; 
        x_dot(i,j) = w(1);
        y_dot(i,j) = w(2); 
    end
end

% Plot DS
f = figure(1); 
screensize = get(groot, 'Screensize');
f.Position = [0.05  * screensize(3), 0.2  * screensize(4), 0.7 * screensize(3), 0.5 * screensize(4)]; 
subplot(1,2,1); hold on;

% plot_ds can either be called with 'streamslice' or 'streamline' for the
% plotting type of the DS (see function definition at the end of the file)
plot_ds(X, Y, x_dot, y_dot, x01, 'Nominal Linear DS', 'streamslice');


%% ------ Write your code below ------
%  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

%% Exercise 2.1 - Local Spurious Attractor 

% TODO : Uncomment and compute a local spurious attractor

% x1 = ; % Spurious attractor coordinates
% r = ; % influence radius ( r < |x-x1|)
% p = ; % p > 1
% 
% for i=1:nb_gridpoints
%     for j=1:nb_gridpoints
%         local_vec = ; % distance to attractor
%         normal_vec = ;
%         tangent_vec = ;
% 
%         Ex = [ ];
%         Dx = ;
%         M1 =;
% 
%         w = ; 
%         x_dot(i,j) = w(1);
%         y_dot(i,j) = w(2); 
%     end
% end
% 
% % Plot DS
% subplot(1,2,2); hold on; 
% plot_ds(X, Y, x_dot, y_dot, x01, ...
%     'Modulated DS with spurious attractor', 'streamslice', x1);


%  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ &
%% ------ Write your code above ------


%% Plotting functions

function plot_ds(x, y, x_dot, y_dot, x0, titleName, type, x_target)
% PLOT_DS  Plot a dynamical system on a grid.
%   PLOT_DS(X, Y, X_DOT, Y_DOT, X0, TITLENAME, TYPE, X_TARGET) where the 
%   arrays X,Y define the coordinates for X_DOT,Y_DOT and are monotonic and
%   2-D plaid (as if produced by MESHGRID), plots a dynamical system with
%   attractor(s) X0 given as 2xN vector data and name TITLENAME.
%
%   The variable TYPE is one of 'streamslice', 'streamline' and defines the
%   plotting style of the DS.
%
%   The optional variable X_TARGET given as 2xN vector data can be used to
%   plot additional points of interest (e.g. local modulation points).
    
    title(titleName)
    [~, h] = contourf(x, y, sqrt(x_dot.^2 + y_dot.^2), 80);
    set(h, 'LineColor', 'none');
    colormap('summer');
    c_bar = colorbar;
    c_bar.Label.String = 'Absolute velocity';
    if exist('type', 'var')
        if strcmp(type, 'streamline')
            h_stream = streamline(x, y, x_dot, y_dot, x(1:2:end, 1:2:end), y(1:2:end, 1:2:end));
        elseif strcmp(type, 'streamslice')
            h_stream = streamslice(x, y, x_dot, y_dot, 2, 'method', 'cubic');
        else
            error('Unsupported plot type');
        end
    else
        error("Set plot type ('streamline' or 'streamslice')");
    end
    set(h_stream, 'LineWidth', 1);
    set(h_stream, 'color', [0. 0. 0.]);
    set(h_stream, 'HandleVisibility', 'off');
    axis equal;

    scatter(x0(1, :), x0(2, :), 100, 'r*', 'LineWidth', 2);

    if exist('x_target', 'var')
        scatter(x_target(1, :), x_target(2, :), 100, 'bd', 'LineWidth', 2);
    end
end