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
filepath = fileparts(which('ch3_ex1_gmrDS.m'));
cd(filepath);
% Choose to draw data (true) or load dataset (false)
draw_data = false;

if draw_data
    %%  Step 1 - OPTION 1 (DATA DRAWING): Draw 2D Dataset with GUI %%

    run('ch3_ex0_drawData.m');

    % Shift attractor to origin for GMR
    disp('Shifting attractor to origin for GMR')
    Data(1:M,:) = Data(1:M,:) - att;
    Xi_ref = Data(1:M,:);
    x0_all = x0_all - att;
    att = [0; 0];

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2 (GMM FITTING): Fit GMM to Trajectory Data %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-sods-opt')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-thirdparty')));
disp('Optimization running, be patient...');

% 0: Manually set the # of Gaussians
% 1: Do Model Selection with BIC
do_ms_bic = 0;
est_options = [];
est_options.type            = 1;   % GMM Estimation Alorithm Type
if do_ms_bic
    est_options.maxK        = 15;  % Maximum Gaussians for Type 1/2
    est_options.do_plots    = 1;   % Plot Estimation Statistics
    est_options.fixed_K     = [];   % Fix K and estimate with EM
    est_options.sub_sample  = 1;   % Size of sub-sampling of trajectories 
    
    [Priors0, Mu0, Sigma0] = fit_gmm([Xi_ref; Xi_dot_ref], [], est_options);
    nb_gaussians = length(Priors0);
else
    % Select manually the number of Gaussian components
    nb_gaussians = 4;
end

% Finding an initial guess for GMM's parameter
[Priors0, Mu0, Sigma0] = initialize_SEDS([Xi_ref; Xi_dot_ref], nb_gaussians);


%  Visualize Gaussian Components and labels on clustered trajectories
% Plot Initial Estimate 
[~, est_labels] =  my_gmm_cluster([Xi_ref; Xi_dot_ref], Priors0, Mu0, Sigma0, 'hard', []);

% Visualize Estimated Parameters
[h_gmm]  = visualizeEstimatedGMM(Xi_ref, Priors0, Mu0(1:M,:), Sigma0(1:M,1:M,:), est_labels, est_options);
title('GMM PDF contour ($\theta_{\gamma}=\{\pi_k,\mu^k,\Sigma^k\}$). Initial Estimate','Interpreter','LaTex');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 3 (DS ESTIMATION): UNCONSTRAINED GMR  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear options;
ds_gmr = @(x) GMR_SEDS(Priors0, Mu0, Sigma0, x - repmat(att, [1 size(x,2)]), 1:M, M+1:2*M);

%%%%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
ds_plot_options = [];
ds_plot_options.sim_traj  = 1;            % To simulate trajectories from x0_all
ds_plot_options.x0_all    = x0_all;       % Intial Points

disp('Visualization loading, be patient...');
[hd, hs, hr, x_sim] = visualizeEstimatedDS(Data(1:M,:), ds_gmr, ds_plot_options);
limits = axis;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 4 (Evaluation): Compute Metrics and Visualize Velocities %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
disp('--------------------')

% Compute RMSE on training data
rmse = mean(rmse_error(ds_gmr, Xi_ref, Xi_dot_ref));
fprintf('Unconstrained GMR-based DS got velocity RMSE on training set: %d \n', rmse);

% Compute e_dot on training data
edot = mean(edot_error(ds_gmr, Xi_ref, Xi_dot_ref));
fprintf('Unconstrained GMR-based DS got velocity deviation (e_dot) on training set: %d \n', edot);

% Compute DTWD between train trajectories and reproductions
if ds_plot_options.sim_traj
    nb_traj       = size(x_sim, 3);
    ref_traj_leng = size(Xi_ref, 2) / nb_traj;
    dtwd = zeros(1, nb_traj);
    for n=1:nb_traj
        start_id = round(1 + (n-1) * ref_traj_leng);
        end_id   = round(n * ref_traj_leng);
        dtwd(1,n) = dtw(x_sim(:,:,n)', Data(1:M,start_id:end_id)', 20);
    end
end
fprintf('Unconstrained GMR-based DS got reproduction DTWD on training set: %2.4f +/- %2.4f \n', mean(dtwd), std(dtwd));

% Compare Velocities from Demonstration vs DS
disp('Visualization loading, be patient...');
h_vel = visualizeEstimatedVelocities(Data, ds_gmr);
