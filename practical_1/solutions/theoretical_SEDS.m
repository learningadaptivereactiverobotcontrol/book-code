%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a demonstration script using SEDS to learn 3D trajectories.
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

load('theoretical_DS_dataset.mat'); 
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


%% Step 2a (GMM FITTING): Fit GMM to Trajectory Data %

% 'seds-init': follows the initialization given in the SEDS code
% 0: Manually set the # of Gaussians
% 1: Do Model Selection with BIC
do_ms_bic = 0;

est_options = [];
est_options.type        = 1;   % GMM Estimation Alorithm Type
est_options.maxK        = 10;  % Maximum Gaussians for Type 1/2
est_options.do_plots    = 1;   % Plot Estimation Statistics
est_options.fixed_K     = [];  % Fix K and estimate with EM
est_options.sub_sample  = 1;   % Size of sub-sampling of trajectories 

if do_ms_bic
    [Priors0, Mu0, Sigma0] = fit_gmm([Xi_ref; Xi_dot_ref], [], est_options);
    nb_gaussians = length(Priors0);
else
    % Select manually the number of Gaussian components
    % Should be at least K=2, so that one is placed on around the attractor

    % We force it to K=1 because the dataset was constructed from one
    % linear DS
    nb_gaussians = 1;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2b (Optional - Initialize params for SEDS Solver): Initialize GMM parameters %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Run Algorithm 1 from Chapter 3 (Get an initial guess by deforming Sigma's)
[Priors0, Mu0, Sigma0] = initialize_SEDS([Xi_ref; Xi_dot_ref],nb_gaussians);

%%  Visualize Gaussian Components and labels on clustered trajectories
% Plot Initial Estimate 
[~, est_labels] =  my_gmm_cluster([Xi_ref; Xi_dot_ref], Priors0, Mu0, Sigma0, 'hard', []);

% Visualize Estimated Parameters
[h_gmm]  = visualizeEstimatedGMM(Xi_ref,  Priors0, Mu0(1:M,:), Sigma0(1:M,1:M,:), est_labels, est_options);
title('GMM $\Theta_{GMR}=\{\pi_k,\mu^k,\Sigma^k\}$ Initial Estimate','Interpreter','LaTex');
%axis([-0.7 0.9 -0.8 0.8 -0.3 1.2])
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%  Step 3 (DS ESTIMATION): RUN SEDS SOLVER  %%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear options;
options.tol_mat_bias  = 10^-4;    % A very small positive scalar to avoid
                                  % instabilities in Gaussian kernel [default: 10^-1]                             
options.display       = 1;        % An option to control whether the algorithm
                                  % displays the output of each iterations [default: true]                            
options.tol_stopping  = 10^-10;   % A small positive scalar defining the stoppping
                                  % tolerance for the optimization solver [default: 10^-10]
options.max_iter      = 1500;      % Maximum number of iteration for the solver [default: i_max=1000]

%options.objective     = 'likelihood';  % 'mse'|'likelihood'
options.objective     = 'mse';       % 'mse'|'likelihood'

sub_sample            = 1;             % sub-sample trajectories by this factor

% Running SEDS optimization solver
[Priors, Mu, Sigma]= SEDS_Solver(Priors0, Mu0, Sigma0, [Xi_ref(:,1:sub_sample:end); Xi_dot_ref(:,1:sub_sample:end)],options); 
clear ds_seds
ds_seds = @(x) GMR_SEDS(Priors, Mu, Sigma,x - repmat(att, [1, size(x,2)]), 1:M, M+1:2*M);

% Computing the theoretical linear DS from SEDS
% !! ONLY WORKS WITH K=1 !!
A_theoric = Sigma(4:6, 1:3) / Sigma(1:3, 1:3)
b_theoric = Mu(4:6) - A_theoric * Mu(1:3)
%% %%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
ds_plot_options = [];
ds_plot_options.sim_traj  = 1;            % To simulate trajectories from x0_all
ds_plot_options.x0_all    = x0_all;       % Intial Points
ds_plot_options.init_type = 'ellipsoid';  % For 3D DS, to initialize streamlines
                                          % 'ellipsoid' or 'cube'  
ds_plot_options.nb_points = 30;           % No of streamlines to plot (3D)
ds_plot_options.plot_vol  = 0;            % Plot volume of initial points (3D)

[hd, hs, hr, x_sim] = visualizeEstimatedDS(Data(1:M,:), ds_seds, ds_plot_options);
scatter(att(1),att(2),150,[0, 0, 0],'d','Linewidth',2); hold on;
limits = axis;
switch options.objective
    case 'mse'        
        title('SEDS $J(\theta_{GMR})$=MSE', 'Interpreter','LaTex','FontSize',30)
    case 'likelihood'
        title('SEDS $J(\theta_{GMR})$=Likelihood', 'Interpreter','LaTex','FontSize',30)
end
set(gcf,'Position',[158, 389, 446, 348])

if M == 2
    legend('Dataset trajectories', 'Learned trajectories')
elseif M == 3
    legend('Dataset trajectories', 'Learned DS')
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Step 5 (Evaluation): Compute Metrics and Visualize Velocities %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute Errors

% Compute RMSE on training data
rmse = mean(rmse_error(ds_seds, Xi_ref, Xi_dot_ref));
fprintf('SEDS got prediction RMSE on training set: %d \n', rmse);

% Compute e_dot on training data
edot = mean(edot_error(ds_seds, Xi_ref, Xi_dot_ref));
fprintf('SEDS got prediction e_dot on training set: %d \n', edot);

% Compute DTWD between train trajectories and reproductions
if ds_plot_options.sim_traj
    nb_traj       = size(x_sim,3);
    ref_traj_leng = size(Xi_ref, 2) / nb_traj;
    dtwd = zeros(1, nb_traj);
    for n=1:nb_traj
        start_id = round(1+ (n - 1) * ref_traj_leng);
        end_id   = round(n * ref_traj_leng);
        dtwd(1,n) = dtw(x_sim(:,:,n)', Data(1:M,start_id:end_id)', 20);
    end
end
fprintf('SEDS got reproduction DTWD on training set: %2.4f +/- %2.4f \n', mean(dtwd),std(dtwd));

% Compare Velocities from Demonstration vs DS
h_vel = visualizeEstimatedVelocities(Data, ds_seds);


%% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%
%%         ADD YOUR CODE ABOVE TO LEARN 3D DS      %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

%% Save DS for simulation using 'DS_control.m'
usingSEDS = true;

if usingSEDS
    ds_control = @(x) ds_seds(x - attractor);
    save(fullfile('..', 'ds_control.mat'), "ds_control", "attractor", "Priors", "Mu", "Sigma", "att", "M")
else
    ds_control = @(x) ds_lpv(x - attractor);
    save(fullfile('..', 'ds_control.mat'), "ds_control", "attractor", "ds_gmm", "A_k", "b_k", "att")
end
