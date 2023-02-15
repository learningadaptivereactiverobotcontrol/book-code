%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a demonstration script using LPVDS to learn 3D trajectories.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Import dependencies
close all; clear; clc
filepath = fileparts(which('MPC_LPVDS.m'));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-ds-opt')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-sods-opt')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-thirdparty')));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, '..', 'dataset')));
cd(filepath);

%% Load and convert 3D dataset
% Import from the dataset folder either:
% - 'demonstration_dataset.mat'
% - 'MPC_train_dataset.mat'
% - 'MPC_test_dataset.mat'
% - 'theoretical_DS_dataset.mat'

load('MPC_train_dataset.mat'); 
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



%% (GMM FITTING): Fit GMM to Trajectory Data %
%%%%%%%%%%%%%%%%%% GMM Estimation Algorithm %%%%%%%%%%%%%%%%%%%%%%

% 0: Physically-Consistent Non-Parametric (Collapsed Gibbs Sampler)
% 1: GMM-EM Model Selection via BIC
% 2: CRP-GMM (Collapsed Gibbs Sampler)
est_options = [];
est_options.type             = 2;   % GMM Estimation Algorithm Type 

% If algo 1 selected:
est_options.maxK             = 20;  % Maximum Gaussians for Type 1
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
%% Give acces to Lasa developper if you are on mac
ControlFlag = readlines('../mac_setup/ControlFlag.txt');
if ismac && (ControlFlag(1) == "LASADEVELOP = True") && (est_options.type ==0)
    disp("Open Acces to LASA developper ")
    system("sudo spctl --master-disable");
end
% Fit GMM to Trajectory Data
[Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);
%% Generate GMM data structure for DS learning
clear ds_gmm; ds_gmm.Mu = Mu; ds_gmm.Sigma = Sigma; ds_gmm.Priors = Priors; 

% (Recommended!) Step 2.1: Dilate the Covariance matrices that are too thin
%This is recommended to get smoother streamlines/global dynamics
adjusts_C  = 1;
if adjusts_C  == 1
    tot_dilation_factor = 1; 
    rel_dilation_fact = 0.75;
    ds_gmm.Sigma = adjust_Covariances(ds_gmm.Priors, ds_gmm.Sigma, tot_dilation_factor, rel_dilation_fact);
end

%  Visualize Gaussian Components and labels on clustered trajectories 
% Extract Cluster Labels
[~, est_labels] =  my_gmm_cluster(Xi_ref, ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, 'hard', []);

% Visualize Estimated Parameters
visualizeEstimatedGMM(Xi_ref, ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, est_labels, est_options);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 3 (DS ESTIMATION): ESTIMATE SYSTEM DYNAMICS MATRICES  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% DS OPTIMIZATION OPTIONS %%%%%%%%%%%%%%%%%%%%%%
% Type of constraints/optimization 
constr_type = 0;      % 0:'convex':     A' + A < 0 (Proposed in paper)
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
    ds_lpv = @(x) lpv_ds(x-repmat(att, [1, size(x,2)]), ds_gmm, A_k, b_k);
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 4 (Evaluation): Compute Metrics and Visualize Velocities %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute RMSE on training data
rmse = mean(rmse_error(ds_lpv, Xi_ref, Xi_dot_ref));
fprintf('LPV-DS with (O%d), got prediction RMSE on training set: %d \n', constr_type+1, rmse);

% Compute e_dot on training data
edot = mean(edot_error(ds_lpv, Xi_ref, Xi_dot_ref));
fprintf('LPV-DS with (O%d), got e_dot on training set: %d \n', constr_type+1, edot);

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


%% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
%%         ADD YOUR CODE ABOVE TO LEARN 3D DS      %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

%% Save DS for simulation using 'DS_control.m'
usingSEDS = false;

if usingSEDS
    ds_control = @(x) ds_seds(x - attractor);
    save(fullfile('..', 'ds_control.mat'), "ds_control", "attractor", "Priors", "Mu", "Sigma", "att", "M")
else
    ds_control = @(x) ds_lpv(x - attractor);
    save(fullfile('..', 'ds_control.mat'), "ds_control", "attractor", "ds_gmm", "A_k", "b_k", "att")
end

if ismac && (ControlFlag(1)== "LASADEVELOP = True") && (est_options.type ==0)
    disp("All access are restablished")
    system("sudo spctl --master-enable");
    writelines("LASADEVLOP = False",'../ControlFlag.txt')
end