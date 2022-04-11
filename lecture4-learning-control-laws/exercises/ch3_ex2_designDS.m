%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for Chapter 3 of:                                       %
% "Robots that can learn and adapt" by Billard, Mirrazavi and Figueroa.   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Nadia Figueroa                                                 %
% email:   nadia.figueroafernandez@epfl.ch                                %
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
%% %%%%%%%%%%% Generate a linear or non-linear DS  %%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('ch3_ex2_designDS.m'));
cd(filepath);
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-ds-opt')));

%%%%%%%%% Example 1 of Chapter 3 %%%%%%%%%
ds_case = 2; % '1'=A_1, '2'=A_2, '3'= 0.5*A_1 + 0.5*A_2 
A1 = [-1 -10; 1 -1];
A2 = [-1 1; -10 -1];


% identical diagonal
A1 = [-2 0; 0 -2];
% distinct diagonal
A1 = [-2 0; 0 -4];
A1 = [-4 0; 0 -2];

% full complex
A1 = [-2 4; -4 -6];

A1 = [-4 2; -2 -4];

% full complex
% A1 = [-2 -4; -0.5 -3];
% A1 = [-eps/(2*p1) -1; p1/p2 -eps/(2*p2)]
% P   = [2 0; 0 2]

p1 = 2; p2 = 4;
P = [p1 0; 0 p2]
Q = A1'*P + P*A1
eig(Q)

switch ds_case
    case 1
        ds_fun = @(x)(A1*x);
        A1_eig = eig(A1);
        A_eigs = eig(0.5*(A1+A1'));
        ds_title = '$\dot{x}=A^1(x-x^*)$';
        
    case 2
        ds_fun = @(x)(A2*x);
        A2_eig = eig(A2)
        A_eigs = eig(0.5*(A2+A2'))
        ds_title = '$\dot{x}=A^2(x-x^*)$';
        
    case 3
        A_sum = (0.5*A1 + 0.5*A2);
        ds_fun = @(x)(A_sum*x);
        A_eigs = eig(0.5*(A_sum+A_sum'))
        ds_title = '$\dot{x}=(0.5A^1 + 0.5A^2)(x-x^*)$';
end

%% %%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
fig1 = figure('Color', [1 1 1]);
fig_limits = [-1 1 -1 1];
[hs] = plot_ds_model(fig1, ds_fun, [0 0]', fig_limits, 'high'); hold on;
[ha] = scatter(0, 0, 200, 'kd', 'filled'); hold on
[ha] = text(0, -0.25, '$x^*$', 'Interpreter', 'LaTex', 'FontSize', 30, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 1], 'EdgeColor', [0 0 0]); hold on
axis(fig_limits)
box on
grid on
xlabel('$x_1$', 'Interpreter', 'LaTex', 'FontSize', 35);
ylabel('$x_2$', 'Interpreter', 'LaTex', 'FontSize', 35);
title(ds_title, 'Interpreter', 'LaTex', 'FontSize', 35);
