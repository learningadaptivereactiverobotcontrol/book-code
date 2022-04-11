%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for  Chapter 3 of:                                      %
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 1 (DATA GENERATION): Draw 2D data with GUI or load dataset %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('ch3_ex1_ffnnDS.m'));
cd(filepath);

% Choose to draw data (true) or load dataset (false)
draw_data = false;

if draw_data
    %%  Step 1 - OPTION 1 (DATA DRAWING): Draw 2D Dataset with GUI %%

    run('ch3_ex0_drawData.m');

else
    %%  Step 1 - OPTION 2 (DATA LOADING): Load Motions from LASA Handwriting Dataset %%
    addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-ds-opt')));

    % Select one of the motions from the LASA Handwriting Dataset
    sub_sample      = 2; % Each trajectory has 1000 samples when set to '1'
    nb_trajectories = 7; % Maximum 7, will select randomly if <7
    [Data, Data_sh, att, x0_all, ~, dt] = load_LASA_dataset_DS(sub_sample, nb_trajectories);
    
    % Position/Velocity Trajectories
    vel_samples = 15; vel_size = 0.5; 
    [h_data, ~, ~] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);
    
    % Extract Position and Velocities
    M          = size(Data,1) / 2;    
    Xi_ref     = Data_sh(1:M,:);
    Xi_dot_ref = Data_sh(M+1:end,:);
    
end
clearvars -except filepath M Xi_ref Xi_dot_ref x0_all Data att

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2 (NN-DS ESTIMATION): ESTIMATE f(x) with NN (no stability) %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-thirdparty')));
disp('Optimization running, be patient...');

% Train a Feed-Forward Neural Network as a function approximator for f(x)

% Generate network structure with random weights
% 1 hidden layer
% ds_neuralNet  =  feedforwardnet(M*3);

% 2 hidden layers
ds_neuralNet  =  feedforwardnet([M*3 M*2]);

% 3 hidden layers
% ds_neuralNet  =  feedforwardnet([M*5 M*3 M*2]);

%% Train network using Levenberg-Marquardt Back-Propagation
[ds_neuralNet,tr] = train(ds_neuralNet, Xi_ref, Xi_dot_ref);
%nntraintool('close');

% Function handle for neural net evaluation
ds_nn  = @(x)ds_neuralNet(x);

%%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
ds_plot_options = [];
ds_plot_options.sim_traj  = 1;            % To simulate trajectories from x0_all
ds_plot_options.x0_all    = x0_all;       % Intial Points

disp('Visualization loading, be patient...');
[hd, hs, hr, x_sim] = visualizeEstimatedDS(Xi_ref, ds_nn, ds_plot_options);
limits = axis;
title('Unconstrained Feed Forward NN-based DS ', 'Interpreter', 'LaTex', 'FontSize', 20)
h_att = scatter(att(1), att(2), 150, [0 0 0], 'd', 'Linewidth', 2); hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 4 (Evaluation): Compute Metrics and Visualize Velocities %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
disp('--------------------')

% Compute RMSE on training data
rmse = mean(rmse_error(ds_nn, Xi_ref, Xi_dot_ref));
fprintf('Unconstrained Feed Forward NN-based DS, got velocity RMSE on training set: %d \n', rmse);

% Compute e_dot on training data
edot = mean(edot_error(ds_nn, Xi_ref, Xi_dot_ref));
fprintf('Unconstrained Feed Forward NN-based DS, got velocity deviation (e_dot) on training set: %d \n', edot);

% Compute DTWD between train trajectories and reproductions
if ds_plot_options.sim_traj
    nb_traj       = size(x_sim, 3);
    ref_traj_leng = size(Xi_ref, 2) / nb_traj;
    dtwd = zeros(1, nb_traj);
    for n=1:nb_traj
        start_id = round(1 + (n-1) * ref_traj_leng);
        end_id   = round(n * ref_traj_leng);
        dtwd(1,n) = dtw(x_sim(:,:,n)', Xi_ref(:,start_id:end_id)', 20);
    end
    fprintf('Unconstrained Feed Forward NN-based DS got DTWD of reproduced trajectories: %2.4f +/- %2.4f \n', mean(dtwd),std(dtwd));
end

% Compare Velocities from Demonstration vs DS
disp('Visualization loading, be patient...');
h_vel = visualizeEstimatedVelocities(Data, ds_nn);
