%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Load data from segmentation experiments        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Select trajectories to test
traj_ids = [1 2 3 4 5];
traj_ids = [1 2];
sub_sample = 1;

% Create Data structure for GMM
Data_ee = [];
for l=1:length(traj_ids)
    % Gather Data
    data_ = Data{traj_ids(l)};
    Data_ee = [Data_ee data_(1:sub_sample:end,1:6)'];
end

% Position/Velocity Trajectories
vel_samples = 15; vel_size = 1.05; 
[h_data, h_vel] = plot_reference_trajectories(Data_ee, vel_samples, vel_size);

% Extract Position and Velocities
[M,N]      = size(Data_ee);    
M = M/2;
Xi_ref     = Data_ee(1:M,:);
Xi_dot_ref = Data_ee(M+1:end,:);   

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 2 (GMM FITTING): Fit GMM to Trajectory Data %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% GMM Estimation Algorithm %%%%%%%%%%%%%%%%%%%%%%
% 0: Physically-Consistent Non-Parametric (Collapsed Gibbs Sampler)
% 1: GMM-EM Model Selection via BIC
% 2: CRP-GMM (Collapsed Gibbs Sampler)
est_options = [];
est_options.type             = 0;   % GMM Estimation Algorithm Type   

% If algo 1 selected:
est_options.maxK             = 15;  % Maximum Gaussians for Type 1
est_options.fixed_K          = round(N/2);  % Fix K and estimate with EM for Type 1

% If algo 0 or 2 selected:
est_options.samplerIter      = 100;  % Maximum Sampler Iterations
                                    % For type 0: 20-50 iter is sufficient
                                    % For type 2: >100 iter are needed
                                    
est_options.do_plots         = 1;   % Plot Estimation Statistics
est_options.sub_sample       = 2;   % Size of sub-sampling of trajectories

% Metric Hyper-parameters
est_options.estimate_l       = 1;   % 0/1 Estimate the lengthscale, if set to 1
est_options.l_sensitivity    = 2;   % lengthscale sensitivity [1-10->>100]
                                    % Default value is set to '2' as in the
                                    % paper, for very messy, close to
                                    % self-interescting trajectories, we
                                    % recommend a higher value
est_options.length_scale     = [];  % if estimate_l=0 you can define your own
                                    % l, when setting l=0 only
                                    % directionality is taken into account

%% Fit GMM to Trajectory Data
tic;
[Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);
toc;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 3: Visualize Gaussian Components and labels on clustered trajectories %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extract Cluster Labels
est_K           = length(Priors);
[~, est_labels] =  my_gmm_cluster(Xi_ref, Priors, Mu, Sigma, 'hard', []);

% Visualize Estimated Parameters
[h_gmm]  = visualizeEstimatedGMM(Xi_ref,  Priors, Mu, Sigma, est_labels, est_options);

