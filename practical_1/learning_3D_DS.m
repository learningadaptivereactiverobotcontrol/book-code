%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is a template file to import 3D dataset to learn a DS from
% trajectories, and exporting the DS to be deployed on the real Panda
% robot. 
% You can use fonctions from chapter 3 to learn a DS using SEDS or LPVDS.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Import dependencies
close all; clear; clc
filepath = fileparts(which('learning_3D_DS.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-ds-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-sods-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-thirdparty')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, 'dataset')));
cd(filepath);

%% Load and convert 3D dataset
% Import from the dataset folder either:
% - 'demonstration_dataset.mat'
% - 'MPC_train_dataset.mat'
% - 'MPC_test_dataset.mat'
% - 'theoretical_DS_dataset.mat'

load("theoretical_DS_dataset.mat"); 
nTraj = size(trajectories, 3);
nPoints = size(trajectories, 2);

Data = zeros(6, nTraj*nPoints);
attractor = zeros(3, 1);
x0_all = zeros(3, nTraj);

for i = 1:nTraj
    Data(:,(i-1)*nPoints+1:i*nPoints) = trajectories(:,:,i);
    x0_all(:,i) = trajectories(1:3,1,i);
    attractor = attractor + trajectories(1:3,end,i);
end
attractor = attractor / nTraj;

% Normalizing dataset attractor position
M = size(Data, 1) / 2; 
Data(1:M,:) = Data(1:M,:) - attractor;
x0_all = x0_all - attractor;
att = [0; 0; 0];

% Plot position/velocity Trajectories
vel_samples = 50; vel_size = 0.75; 
[h_data, h_att, ~] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);

% Extract Position and Velocities
M = size(Data,1) / 2;    
Xi_ref = Data(1:M,:);
Xi_dot_ref  = Data(M+1:end,:);   
axis_limits = axis;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%%         ADD YOUR CODE BELOW TO LEARN 3D DS      %%
%% vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%






%% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
%%         ADD YOUR CODE ABOVE TO LEARN 3D DS      %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

%% Save DS for simulation using 'DS_control.m'
usingSEDS = true;

if usingSEDS
    ds_control = @(x) ds_seds(x - attractor);
    save('ds_control.mat', "ds_control", "attractor", "Priors", "Mu", "Sigma", "att", "M")
else
    ds_control = @(x) ds_lpv(x - attractor);
    save('ds_control.mat', "ds_control", "attractor", "ds_gmm", "A_k", "b_k", "att")
end

