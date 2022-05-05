%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
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
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%    Chapter 10 - Dynamical system based compliant control:
%    Programming exercise 1   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example:  2-DOF Robotic arm- impedance controller comparision
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc;
folder = fileparts(which('ch10_ex1.m'));
addpath(genpath(fullfile(folder, '..', 'libraries', 'Impedance_controller')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Dynamic specifications %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.m1=1.0; %% Mass of the first link
robot.m2=0.5; %% Mass of the second link
robot.l1=1.0; %% length of the first link
robot.l2=0.5; %% length of the second link

%% Section 1: Without inertia compensation

% Initial configuration
robot.q1=-deg2rad(90); %% The initial position of the first joint
robot.q2=0;      %% The initial position of the second joint
robot.Dq1=0.0;       %% The initial velocity of the first joint
robot.Dq2=0.0;       %% The initial velocity of the second joint
robot.DDq1=0.0;      %% The initial acceleration of the first joint
robot.DDq2=0.0;      %% The initial acceleration of the second joint

% Impedance parameters
Stiffness=20*eye(2); %% desired stiffness matrix
Damping=5*eye(2);  %% desired damping matrix

%%% Robot initializations %%
External_force=100; %% External perturbation force
robot.q=[robot.q1;robot.q2]; %% Initial position
robot.Dq=[robot.Dq1;robot.Dq2]; %% Initial velocity  
robot.DDq=[robot.DDq1;robot.DDq2]; %% Initial acceleration
robot.tau1=0.0; %% The initial torque of the first joint
robot.tau2=0.0; %% The initial torque of the second joint
robot.tau=[robot.tau1;robot.tau2]; % Initial torques

Impedance_controller_minimal2(robot,Damping,Stiffness,External_force)

%% Section 2: With inertia compensation

% Impedance parameters
Stiffness=20*eye(2); %% desired stiffness matrix
Damping=5*eye(2);  %% desired damping matrix

Impedance_controller_minimal1(robot,Damping,Stiffness,External_force)

%% Section 3: With inertia reshaping (full controller)

% Impedance parameters
Stiffness=20*eye(2); %% desired stiffness matrix
Damping=5*eye(2);  %% desired damping matrix
Inertia=2*eye(2);   %% desired inertia matrix

Impedance_controller_full(robot,Inertia,Damping,Stiffness,External_force)
