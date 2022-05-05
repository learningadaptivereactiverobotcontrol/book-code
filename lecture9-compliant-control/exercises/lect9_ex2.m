
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
%%%  Name of the chapter: Dynamical system based compliant control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%    Chapter 10 - Dynamical system based compliant control:
%    Programming exercise 2 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example:  2-DOF Robotic arm- impedance controller comparision
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc; close all;
folder = fileparts(which('ch10_ex2.m'));
addpath(genpath(fullfile(folder, '..', 'libraries', 'Variable_Impedance_controller')));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Dynamic specifications %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.m1=1.0; %% Mass of the first link
robot.m2=0.5; %% Mass of the second link
robot.l1=1.0; %% length of the first link
robot.l2=0.5; %% length of the second link

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Initial configuration %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.q1=-90*pi/180; %% The initial position of the first joint
robot.q2=0.0;        %% The initial position of the second joint
robot.Dq1=0.0;       %% The initial velocity of the first joint
robot.Dq2=0.0;       %% The initial velocity of the second joint
robot.DDq1=0.0;      %% The initial acceleration of the first joint
robot.DDq2=0.0;      %% The initial acceleration of the second joint

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% External perturbation force %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
External_force=500;

%%% Initializations %%
robot.q=[robot.q1;robot.q2];
robot.Dq=[robot.Dq1;robot.Dq2];
robot.DDq=[robot.DDq1;robot.DDq2];
robot.tau1=0.0;
robot.tau2=0.0;
robot.tau=[robot.tau1;robot.tau2];

%% Section 1: Varying gains between two configurations %%
% Set impedance variables and plot approach 1

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Impedance variables %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
Inertia1 = {};
Damping1 = {};
Stiffness1 = {};
Inertia1{1,1}=1*eye(2);    %% desired inertia matrix 1
Damping1{1,1}=10*eye(2);   %% desired damping matrix 1
Stiffness1{1,1}=25*eye(2);  %% desired stiffness matrix 1

Inertia1{1,2}=1*eye(2);    %% desired inertia matrix 2
Damping1{1,2}=100*eye(2);  %% desired damping matrix 2
Stiffness1{1,2}=2500*eye(2);%% desired stiffness matrix 2

mu1{1,1}=[-1;1]; %% Scheduling parameter 1 at position level; 
mu1{1,2}=[1;1];  %% Scheduling parameter 2 at position level; 

% Plot approach 1
Impedance_controller_approch_one(robot,Inertia1,Damping1,Stiffness1,mu1,External_force)

%% Section 2: Varying gains with time dependence %%
% Set impedance variables and plot approach 2

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Impedance variables %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
Inertia2 = {};
Damping2 = {};
Stiffness2 = {};
Inertia2{1,1}=1*eye(2);    %% desired inertia matrix 1
Damping2{1,1}=10*eye(2);   %% desired damping matrix 1
Stiffness2{1,1}=25*eye(2);  %% desired stiffness matrix 1

Inertia2{1,2}=1*eye(2);    %% desired inertia matrix 2
Damping2{1,2}=100*eye(2);  %% desired damping matrix 2
Stiffness2{1,2}=2500*eye(2);%% desired stiffness matrix 2

mu2{1,1}=[-1;1]; %% Scheduling parameter 1 at position level; 
mu2{1,2}=[1;1];  %% Scheduling parameter 2 at position level; 
mu_DQ{1,1}=[0;0];   %% Scheduling parameter 1 at velocity level; 
mu_DQ{1,2}=[-1;-1]; %% Scheduling parameter 2 at velocity level; 

% Plot Approach 2 
Impedance_controller_approch_two(robot,Inertia2,Damping2,Stiffness2,mu2,mu_DQ,External_force)


%% Section 3: Basic time dependence %%
% Set impedance variables and plot approach 3

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Impedance variables %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
Inertia3=1*eye(2);       %% Constant inertia matrix
Damping_i=zeros(2);     %% initial damping matrix
Stiffness_i=100*eye(2);  %% initial stiffness matrix

Damping_d=100*eye(2);  %% desired damping matrix
Stiffness_d=2500*eye(2);%% desired stiffness matrix

ConvergenceRate = 0.1; % Convergence rate of the damping and the stiffness matrices

% Plot approach 3
Impedance_controller_approch_three(robot,Inertia3,Damping_i,Damping_d, Stiffness_i,Stiffness_d,ConvergenceRate,External_force)

