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
filepath = fileparts(which('ch3_ex1_svrDS.m'));
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2 (SVR-DS ESTIMATION): ESTIMATE f(x) with SVR (no stability) %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-ml-toolbox')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-thirdparty')));
disp('Optimization running, be patient...');
tStart = cputime;

% Preparing data for SVR
X_train   = Xi_ref';
x1_dot_train  = Xi_dot_ref(1,:)';
x2_dot_train  = Xi_dot_ref(2,:)';

% SVR Hyper-parameters
clear svr_options
svr_options.svr_type    = 0;    % 0: epsilon-SVR, 1: nu-SVR
svr_options.C           = 100;   % set the parameter C of C-SVC, epsilon-SVR, and nu-SVR 
svr_options.epsilon     = 1; % set the epsilon in loss function of epsilon-SVR 

% Kernel OPTIONS
svr_options.kernel_type = 2;    % 0: linear: u'*v, 1: polynomial: (gamma*u'*v + coef0)^degree, 2: radial basis function: exp(-gamma*|u-v|^2)
svr_options.sigma       = 1;  %  radial basis function: exp(-gamma*|u-v|^2), gamma = 1/(2*sigma^2)

% Train SVR's for x_1 and x_2 coordinates
clear model_x1 model_x2
[~, model_x1] = svm_regressor(X_train, x1_dot_train, svr_options, []);
[~, model_x2] = svm_regressor(X_train, x2_dot_train, svr_options, []);
ds_svr_x1        = @(x)svm_regressor(x', [], svr_options, model_x1);
ds_svr_x2        = @(x)svm_regressor(x', [], svr_options, model_x2);

% 2D output (x_1+x_2)
ds_svr          = @(x)([ds_svr_x1(x) ds_svr_x2(x)]');

%%%%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
ds_plot_options = [];
ds_plot_options.sim_traj  = 1;            % To simulate trajectories from x0_all
ds_plot_options.x0_all    = x0_all;       % Intial Points

disp('Visualization loading, be patient...');
[hd, hs, hr, x_sim] = visualizeEstimatedDS(Xi_ref, ds_svr, ds_plot_options);
limits = axis;
title('Unconstrained SVR-based DS ', 'Interpreter', 'LaTex', 'FontSize', 20)
h_att = scatter(att(1), att(2), 150, [0 0 0], 'd', 'Linewidth', 2); hold on;
tEnd = cputime - tStart;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 4 (Evaluation): Compute Metrics and Visualize Velocities %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
% Compute RMSE on training data
rmse = mean(rmse_error(ds_svr, Xi_ref, Xi_dot_ref));

% Compute e_dot on training data
edot = mean(edot_error(ds_svr, Xi_ref, Xi_dot_ref));

disp('--------------------')
fprintf('Unconstrained SVR-based DS, got velocity RMSE on training set: %d \n', rmse);
fprintf('Unconstrained SVR-based DS, got velocity deviation (e_dot) on training set: %d \n', edot);
% Display time 
fprintf('Computation Time is %1.2f seconds \n', tEnd);

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
    fprintf('Unconstrained SVR-based DS got DTWD of reproduced trajectories: %2.4f +/- %2.4f \n', mean(dtwd),std(dtwd));
end

%% Compare Velocities from Demonstration vs DS
disp('Visualization loading, be patient...');
evalc('visualizeEstimatedVelocities(Data, ds_svr)');
