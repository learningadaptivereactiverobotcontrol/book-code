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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 1 (DATA GENERATION): Draw 2D data with GUI or load dataset %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;
filepath = fileparts(which('ch3_ex4_lpvDS.m'));
cd(filepath);

% Choose to draw data (true) or load dataset (false)
draw_data = false;

if draw_data
    %%  Step 1 - OPTION 1 (DATA DRAWING): Draw 2D Dataset with GUI %%

    run('ch3_ex0_drawData.m');
else
    %%  Step 1 - OPTION 2 (DATA LOADING): Load Motions from LASA Handwriting Dataset %%
    addpath(genpath(fullfile('..', '..', 'libraries', 'book-ds-opt')));

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
clearvars -except filepath M Xi_ref Xi_dot_ref x0_all Data Data_sh att

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2 (GMM FITTING): Fit GMM to Trajectory Data %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-sods-opt')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries','book-thirdparty')));

%%%%%%%%%%%%%%%%%% GMM Estimation Algorithm %%%%%%%%%%%%%%%%%%%%%%

% 0: Physically-Consistent Non-Parametric (Collapsed Gibbs Sampler)
% 1: GMM-EM Model Selection via BIC
% 2: CRP-GMM (Collapsed Gibbs Sampler)
est_options = [];
est_options.type             = 1;   % GMM Estimation Algorithm Type 

% If algo 1 selected:
est_options.maxK             = 10;  % Maximum Gaussians for Type 1
est_options.fixed_K          = [];  % Fix K and estimate with EM for Type 1

% If algo 0 or 2 selected:
est_options.samplerIter      = 50;  % Maximum Sampler Iterations
                                    % For type 0: 20-50 iter is sufficient
                                    % For type 2: >100 iter are needed
                                    
est_options.do_plots         = 1;   % Plot Estimation Statistics
% Size of sub-sampling of trajectories
% 1/2 for 2D datasets, >2/3 for real
nb_data = length(Data);
sub_sample = 1;
if nb_data > 500
    sub_sample = 2;
elseif nb_data > 1000
    sub_sample = 3;
end
est_options.sub_sample       = sub_sample;

% Metric Hyper-parameters
est_options.estimate_l       = 1;   % '0/1' Estimate the lengthscale, if set to 1
est_options.l_sensitivity    = 2;   % lengthscale sensitivity [1-10->>100]
                                    % Default value is set to '2' as in the
                                    % paper, for very messy, close to
                                    % self-intersecting trajectories, we
                                    % recommend a higher value
est_options.length_scale     = [];  % if estimate_l=0 you can define your own
                                    % l, when setting l=0 only
                                    % directionality is taken into account

% Fit GMM to Trajectory Data
[Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);

%% Generate GMM data structure for DS learning
clear ds_gmm; ds_gmm.Mu = Mu; ds_gmm.Sigma = Sigma; ds_gmm.Priors = Priors; 

% (Recommended!) Step 2.1: Dilate the Covariance matrices that are too thin
% This is recommended to get smoother streamlines/global dynamics
adjusts_C  = 1;
if adjusts_C  == 1
    if M == 2
        tot_dilation_factor = 1; rel_dilation_fact = 0.25;
    elseif M == 3
        tot_dilation_factor = 1; rel_dilation_fact = 0.75;
    end
    Sigma_ = adjust_Covariances(ds_gmm.Priors, ds_gmm.Sigma, tot_dilation_factor, rel_dilation_fact);
    ds_gmm.Sigma = Sigma_;
end

%  Visualize Gaussian Components and labels on clustered trajectories 
% Extract Cluster Labels
[~, est_labels] =  my_gmm_cluster(Xi_ref, ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, 'hard', []);

% Visualize Estimated Parameters
visualizeEstimatedGMM(Xi_ref, ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, est_labels, est_options);
title('GMM PDF contour ($\theta_{\gamma}=\{\pi_k,\mu^k,\Sigma^k\}$). Initial Estimate','Interpreter','LaTex');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 3 (DS ESTIMATION): ESTIMATE SYSTEM DYNAMICS MATRICES  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% DS OPTIMIZATION OPTIONS %%%%%%%%%%%%%%%%%%%%%% 
% Type of constraints/optimization 
constr_type = 2;      % 0:'convex':     A' + A < 0 (Proposed in paper)
                      % 1:'non-convex': A'P + PA < 0 (Sina's Thesis approach - not suitable for 3D)
                      % 2:'non-convex': A'P + PA < -Q given P (Proposed in paper)                                 
init_cvx    = 1;      % 0/1: initialize non-cvx problem with cvx     

if constr_type == 0 || constr_type == 1
    P_opt = eye(M);
else
    % P-matrix learning
%     [Vxf] = learn_wsaqf(Data,0,att);
   
    % (Data shifted to the origin)
    % Assuming origin is the attractor (works better generally)
    [Vxf] = learn_wsaqf(Data);
    P_opt = Vxf.P;
end

%%%%%%%%  LPV system sum_{k=1}^{K}\gamma_k(xi)(A_kxi + b_k) %%%%%%%%  
if constr_type == 1
    [A_k, b_k, P_est] = optimize_lpv_ds_from_data(Data, zeros(M,1), constr_type, ds_gmm, P_opt, init_cvx);
    ds_lpv = @(x) lpv_ds(x-repmat(att, [1 size(x,2)]), ds_gmm, A_k, b_k);
else
    [A_k, b_k, P_est] = optimize_lpv_ds_from_data(Data, att, constr_type, ds_gmm, P_opt, init_cvx);
    ds_lpv = @(x) lpv_ds(x, ds_gmm, A_k, b_k);
end

%% %%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
ds_plot_options = [];
ds_plot_options.sim_traj  = 1;            % To simulate trajectories from x0_all
ds_plot_options.x0_all    = x0_all;       % Intial Points
ds_plot_options.init_type = 'ellipsoid';  % For 3D DS, to initialize streamlines
                                          % 'ellipsoid' or 'cube'
ds_plot_options.nb_points = 30;           % No of streamlines to plot (3D)
ds_plot_options.plot_vol  = 0;            % Plot volume of initial points (3D)

[hd, hs, hr, x_sim] = visualizeEstimatedDS(Xi_ref, ds_lpv, ds_plot_options);
limits = axis;
switch constr_type
    case 0
        title('GMM-based LPV-DS with QLF', 'Interpreter', 'LaTex', 'FontSize', 20)
    case 1
        title('GMM-based LPV-DS with P-QLF (v0) ', 'Interpreter', 'LaTex', 'FontSize', 20)
    case 2
        title('GMM-based LPV-DS with P-QLF', 'Interpreter', 'LaTex', 'FontSize', 20)
end

if M == 2
    legend('Dataset trajectories', 'Learned trajectories')
elseif M == 3
    legend('Dataset trajectories', 'Learned DS')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 4 (Optional - Stability Check 2D-only): Plot Lyapunov Function and derivative  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Type of plot
contour = 1; % 0: surf, 1: contour
clear lyap_fun_comb lyap_der 

switch constr_type
    case 0 
        P = eye(2);
        title_string = {'$V(\xi) = (\xi-\xi^*)^T(\xi-\xi^*)$'};
    case 1
        P = P_est;
        title_string = {'$V(\xi) = (\xi-\xi^*)^TP(\xi-\xi^*)$'};
    case 2
        P = P_opt;
        title_string = {'$V(\xi) = (\xi-\xi^*)^TP(\xi-\xi^*)$'};
end

if M == 2
    % Lyapunov function
    lyap_fun = @(x)lyapunov_function_PQLF(x, att, P);
    
    % Derivative of Lyapunov function (gradV*f(x))
    lyap_der = @(x)lyapunov_derivative_PQLF(x, att, P, ds_lpv);
    title_string_der = {'Lyapunov Function Derivative $\dot{V}(\xi)$'};
    
    % Plots
    h_lyap     = plot_lyap_fct(lyap_fun, contour, limits,  title_string, 0);
    hd = scatter(Data(1,:), Data(2,:), 10, [1 1 0], 'filled'); hold on;
    h_lyap_der = plot_lyap_fct(lyap_der, contour, limits,  title_string_der, 1);
    hd = scatter(Xi_ref(1,:), Xi_ref(2,:), 10, [1 1 0], 'filled'); hold on;
else
    clc;
    fprintf(2,'Lyapunov Function Plotting: Not possible for 3D data.\n')    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 5 (Evaluation): Compute Metrics and Visualize Velocities %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
disp('--------------------')

% Compute RMSE on training data
rmse = mean(rmse_error(ds_lpv, Xi_ref, Xi_dot_ref));
fprintf('LPV-DS with (O%d), got velocity RMSE on training set: %d \n', constr_type+1, rmse);

% Compute e_dot on training data
edot = mean(edot_error(ds_lpv, Xi_ref, Xi_dot_ref));
fprintf('LPV-DS with (O%d), got velocity deviation (e_dot) on training set: %d \n', constr_type+1, edot);

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
    fprintf('LPV-DS got DTWD of reproduced trajectories: %2.4f +/- %2.4f \n', mean(dtwd), std(dtwd));
end

% Compare Velocities from Demonstration vs DS
h_vel = visualizeEstimatedVelocities(Data, ds_lpv);
