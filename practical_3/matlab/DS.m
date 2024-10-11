%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Aude Billard
%   email:   aude.billard@epfl.ch
%   website: lasa.epfl.ch
%    
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef DS < handle
    %DS - Class to store, learn and compute DS 
    
    properties
        dsControl
        dsUnmodulated
        algoName
        myWorld
        
        doPlots
        doModulation
        
        attractor
    end
    
    methods
        function self = DS(algoName, myWorld, do_plots, do_modulation)
            % Make it possible to set hyperparameters here ?
            % struct lpvds and struct seds and store all params there
            % must specify algo when instantiating DS
            self.algoName = algoName;
            if exist('do_plots', 'var') == 0
                self.doPlots = 1;
            else
                self.doPlots = do_plots;
            end

            if exist('do_modulation', 'var') == 0
                self.doModulation = 0;
            else
                self.doModulation = do_modulation;
            end

            self.myWorld = myWorld;
            self.dsControl = @(x) [0;0;0];
            self.dsUnmodulated = @(x) [0;0;0];

            self.attractor = [0.0; 0.0; 0.0];
        end
        
        function learnLPVDS(self, trajectories)
            % Learn DS using LPVDS
            
            disp('Started learning with LPVDS')

            % Normalize trajectories
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
            
            if self.doPlots
                % Plot position/velocity Trajectories
                vel_samples = 50; vel_size = 0.75; 
                [h_data, h_att, ~] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);
            end
            
            % Extract Position and Velocities
            M = size(Data,1) / 2;    
            Xi_ref = Data(1:M,:);
            Xi_dot_ref  = Data(M+1:end,:);   
            axis_limits = axis;

            %% (GMM FITTING): Fit GMM to Trajectory Data %
            %%%%%%%%%%%%%%%%%% GMM Estimation Algorithm %%%%%%%%%%%%%%%%%%%%%%
            
            % 0: Physically-Consistent Non-Parametric (Collapsed Gibbs Sampler)
            % 1: GMM-EM Model Selection via BIC
            % 2: CRP-GMM (Collapsed Gibbs Sampler)
            est_options = [];
            est_options.type             = 0;   % GMM Estimation Algorithm Type 
            
            % If algo 1 selected:
            est_options.maxK             = 20;  % Maximum Gaussians for Type 1
            est_options.fixed_K          = [];  % Fix K and estimate with EM for Type 1
            
            % If algo 0 or 2 selected:
            est_options.samplerIter      = 20;  % Maximum Sampler Iterations
                                                % For type 0: 20-50 iter is sufficient
                                                % For type 2: >100 iter are needed
                                                
            est_options.do_plots         = self.doPlots;   % Plot Estimation Statistics
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
            %This is recommended to get smoother streamlines/global dynamics
            adjusts_C  = 1;
            if adjusts_C  == 1
                tot_dilation_factor = 1; 
                rel_dilation_fact = 0.75;
                ds_gmm.Sigma = adjust_Covariances(ds_gmm.Priors, ds_gmm.Sigma, tot_dilation_factor, rel_dilation_fact);
            end
            
            if self.doPlots
                %  Visualize Gaussian Components and labels on clustered trajectories 
                % Extract Cluster Labels
                [~, est_labels] =  my_gmm_cluster(Xi_ref, ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, 'hard', []);
            
                % Visualize Estimated Parameters
                visualizeEstimatedGMM(Xi_ref, ds_gmm.Priors, ds_gmm.Mu, ds_gmm.Sigma, est_labels, est_options);
            end

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
                ds_lpv = @(x) lpv_ds(x-repmat(att, [1, size(x,2)]), ds_gmm, A_k, b_k);
            else
                [A_k, b_k, P_est] = optimize_lpv_ds_from_data(Data, att, constr_type, ds_gmm, P_opt, init_cvx);
                ds_lpv = @(x) lpv_ds(x, ds_gmm, A_k, b_k);
            end
            

            %% %%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
            
            if self.doPlots
                
                % Fill in plotting options
                ds_plot_options = [];
                ds_plot_options.sim_traj  = 1;            % To simulate trajectories from x0_all
                ds_plot_options.x0_all    = x0_all;       % Intial Points
                ds_plot_options.init_type = 'ellipsoid';  % For 3D DS, to initialize streamlines
                                                          % 'ellipsoid' or 'cube'
                ds_plot_options.nb_points = 10;           % No of streamlines to plot (3D)
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

                % Set plot options
                opt_sim = []; 
                opt_sim.plot  = 0;
                opt_sim.dt = 2e-2;%3e-3;
                opt_sim.i_max = 1500;
                
                openLoopTraj= 40; % number of open loop trajectories to disply 
                
                % Plot DS open-loop path starting at random points
                axisLimit = 0.9*self.myWorld.axisLimit;
                for iTraj = 1:openLoopTraj
                    randomPosition = rand(3, 1);
                    randomPosition(1) = 0.8*(randomPosition(1)*(axisLimit(2)-axisLimit(1)) + axisLimit(1));
                    randomPosition(2) = 0.9*(randomPosition(2)*(axisLimit(4)-axisLimit(3)) + axisLimit(3));
                    randomPosition(3) = 0.6*(randomPosition(3)*(axisLimit(6)-axisLimit(5)) + axisLimit(5));
                    scatter3(randomPosition(1), randomPosition(2), randomPosition(3), 25, 'MarkerEdgeColor','k','MarkerFaceColor',[0 .25 1], 'DisplayName','Learned DS random starting point');
                    [~, x_sim, xdot_sim, time, ~] = evalc('Simulation(randomPosition , [], ds_lpv, opt_sim)');

                    % Augment x0_all to plot mroe than itnital demonstrations
                    hold on;
                    plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'Color', [0 .25 1], 'DisplayName','Open loop trajectories');

                    % interpolate to reduce number of quiver arrows 
                    number_of_arrows = 50;
                    interp_time = linspace(time(1), time(end), number_of_arrows);
                    x_interp = zeros(size(x_sim, 1), number_of_arrows);
                    xdot_interp = zeros(size(x_sim, 1), number_of_arrows);
                    for k = 1:size(x_sim, 1) %for each dimension
                        x_interp(k,:) = interp1(time, x_sim(k,:), interp_time);
                        xdot_interp(k,:) = interp1(time, xdot_sim(k,:), interp_time);
                    end

                    q = quiver3(x_interp(1,:), x_interp(2,:), x_interp(3,:), xdot_interp(1,:), xdot_interp(2,:), xdot_interp(3,:), ...
                                1 ,'Color', [0 0.25 0.8], 'LineWidth', 0.25, 'MaxHeadSize', 0.1);
                    hold on;

                    %// Compute the magnitude of the vectors
                    mags = sqrt(sum(cat(2, q.UData(:), q.VData(:), ...
                                reshape(q.WData, numel(q.UData), [])).^2, 2));
                    
                    %// Get the current colormap
                    currentColormap = colormap(gca);
                    
                    %// Now determine the color to make each arrow using a colormap
                    clims = num2cell(get(gca, 'clim'));
                    [~, ~, ind] = histcounts(mags, linspace(clims{:}, size(currentColormap, 1)));
                    %[~, ~, ind] = histcounts(mags, size(currentColormap, 1));  
                    
                    %// Now map this to a colormap to get RGB
                    cmap = uint8(ind2rgb(ind(:), currentColormap) * 255);
                    cmap(:,:,4) = 50; % arrow alpha
                    cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);
                    
                    %// We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
                    set(q.Head, ...
                        'ColorBinding', 'interpolated', ...
                        'ColorData', reshape(cmap(1:3,:,:), [], 4).');   %'
                    
                    %// We repeat each color 2 times (using 1:2 below) because each tail has 2 vertices
                    set(q.Tail, ...
                        'ColorBinding', 'interpolated', ...
                        'ColorData', reshape(cmap(1:2,:,:), [], 4).');

                end
                colorbar('eastoutside')
                legend('Demonstration dataset', 'Learned DS', 'Open loop trajectories', 'Location', 'best');


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
            
            if self.doPlots
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
                h_vel = visualizeEstimatedVelocities(Data, self.dsControl);
            end

            %% %%%%%%%% Save DS to object  %%%%%%%%%%%%%%%%%%%%
            self.attractor = attractor;

            if self.doModulation
                self.dsUnmodulated = @(x) ds_lpv(x - attractor);
                self.dsControl = @(x) self.modulatedDS(x);
            else
                self.dsUnmodulated = @(x) ds_lpv(x - attractor);
                self.dsControl = @(x) ds_lpv(x - attractor);
            end

            disp('Finished learning with LPVDS')

        end


        function learnSEDS(self, trajectories)

            % Learn DS using SEDS
            
            disp('Started learning with SEDS')

            % Normalize trajectories
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
            
            if self.doPlots
                % Plot position/velocity Trajectories
                vel_samples = 50; vel_size = 0.75; 
                [h_data, h_att, ~] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);
            end
            
            % Extract Position and Velocities
            M = size(Data,1) / 2;    
            Xi_ref = Data(1:M,:);
            Xi_dot_ref  = Data(M+1:end,:);   
            axis_limits = axis;            
            
            %% Step 2a (GMM FITTING): Fit GMM to Trajectory Data %
            
            % 'seds-init': follows the initialization given in the SEDS code
            % 0: Manually set the # of Gaussians
            % 1: Do Model Selection with BIC
            do_ms_bic = 1;
            
            est_options = [];
            est_options.type        = 1;   % GMM Estimation Alorithm Type
            est_options.maxK        = 15;  % Maximum Gaussians for Type 1/2
            est_options.do_plots    = self.doPlots;   % Plot Estimation Statistics
            est_options.fixed_K     = [];  % Fix K and estimate with EM
            est_options.sub_sample  = 1;   % Size of sub-sampling of trajectories 
            
            if do_ms_bic
                [Priors0, Mu0, Sigma0] = fit_gmm([Xi_ref; Xi_dot_ref], [], est_options);
                nb_gaussians = length(Priors0);
            else
                % Select manually the number of Gaussian components
                % Should be at least K=2, so that one is placed on around the attractor
                nb_gaussians = 12;
            end
            
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%  Step 2b (Optional - Initialize params for SEDS Solver): Initialize GMM parameters %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Run Algorithm 1 from Chapter 3 (Get an initial guess by deforming Sigma's)
            [Priors0, Mu0, Sigma0] = initialize_SEDS([Xi_ref; Xi_dot_ref],nb_gaussians);
            
            %%  Visualize Gaussian Components and labels on clustered trajectories
            if self.doPlots
                % Plot Initial Estimate 
                [~, est_labels] =  my_gmm_cluster([Xi_ref; Xi_dot_ref], Priors0, Mu0, Sigma0, 'hard', []);
                
                % Visualize Estimated Parameters
                [h_gmm]  = visualizeEstimatedGMM(Xi_ref,  Priors0, Mu0(1:M,:), Sigma0(1:M,1:M,:), est_labels, est_options);
                title('GMM $\Theta_{GMR}=\{\pi_k,\mu^k,\Sigma^k\}$ Initial Estimate','Interpreter','LaTex');
                %axis([-0.7 0.9 -0.8 0.8 -0.3 1.2])
            end

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
            options.max_iter      = 300;      % Maximum number of iteration for the solver [default: i_max=1000]
            
            options.objective     = 'likelihood';  % 'mse'|'likelihood'
            %options.objective     = 'mse';       % 'mse'|'likelihood'
            
            sub_sample            = 1;             % sub-sample trajectories by this factor
            
            % Running SEDS optimization solver
            [Priors, Mu, Sigma]= SEDS_Solver(Priors0, Mu0, Sigma0, [Xi_ref(:,1:sub_sample:end); Xi_dot_ref(:,1:sub_sample:end)],options); 
            clear ds_seds
            ds_seds = @(x) GMR_SEDS(Priors, Mu, Sigma,x - repmat(att, [1, size(x,2)]), 1:M, M+1:2*M);
            
            %% %%%%%%%%%%%%    Plot Resulting DS  %%%%%%%%%%%%%%%%%%%
            
            if self.doPlots
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
                %set(gcf,'Position',[158, 389, 446, 348])
                
                % Set plot options
                opt_sim = []; 
                opt_sim.plot  = 0;
                opt_sim.dt = 2e-2;%3e-3;
                opt_sim.i_max = 1500;
                
                openLoopTraj= 40; % number of open loop trajectories to disply 
                number_of_arrows = 40; % number of velocity arrows per trajectory

                % Plot DS open-loop path starting at random points
                axisLimit = 0.9*self.myWorld.axisLimit;
                for iTraj = 1:openLoopTraj
                    randomPosition = rand(3, 1);
                    randomPosition(1) = 0.8*(randomPosition(1)*(axisLimit(2)-axisLimit(1)) + axisLimit(1));
                    randomPosition(2) = 0.9*(randomPosition(2)*(axisLimit(4)-axisLimit(3)) + axisLimit(3));
                    randomPosition(3) = 0.6*(randomPosition(3)*(axisLimit(6)-axisLimit(5)) + axisLimit(5));
                    scatter3(randomPosition(1), randomPosition(2), randomPosition(3), 25, 'MarkerEdgeColor','k','MarkerFaceColor',[0 .25 1], 'DisplayName','Learned DS random starting point');
                    [~, x_sim, xdot_sim, time, ~] = evalc('Simulation(randomPosition , [], ds_seds, opt_sim)');

                    % Augment x0_all to plot mroe than itnital demonstrations
                    hold on;
                    plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'Color', [0 .25 1], 'DisplayName','Open loop trajectories');

                    % interpolate to reduce number of quiver arrows 
                    interp_time = linspace(time(1), time(end), number_of_arrows);
                    x_interp = zeros(size(x_sim, 1), number_of_arrows);
                    xdot_interp = zeros(size(x_sim, 1), number_of_arrows);
                    for k = 1:size(x_sim, 1) %for each dimension
                        x_interp(k,:) = interp1(time, x_sim(k,:), interp_time);
                        xdot_interp(k,:) = interp1(time, xdot_sim(k,:), interp_time);
                    end

                    q = quiver3(x_interp(1,:), x_interp(2,:), x_interp(3,:), xdot_interp(1,:), xdot_interp(2,:), xdot_interp(3,:), ...
                                2 ,'Color', [0 0.25 0.8], 'LineWidth', 0.25, 'MaxHeadSize', 0.1);
                    hold on;

                    %// Compute the magnitude of the vectors
                    mags = sqrt(sum(cat(2, q.UData(:), q.VData(:), ...
                                reshape(q.WData, numel(q.UData), [])).^2, 2));
                    
                    %// Get the current colormap
                    currentColormap = colormap(gca);
                    
                    %// Now determine the color to make each arrow using a colormap
                    clims = num2cell(get(gca, 'clim'));
                    [~, ~, ind] = histcounts(mags, linspace(clims{:}, size(currentColormap, 1)));
                    %[~, ~, ind] = histcounts(mags, size(currentColormap, 1));  
                    
                    %// Now map this to a colormap to get RGB
                    cmap = uint8(ind2rgb(ind(:), currentColormap) * 255);
                    cmap(:,:,4) = 50; % arrow alpha
                    cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);
                    
                    %// We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
                    set(q.Head, ...
                        'ColorBinding', 'interpolated', ...
                        'ColorData', reshape(cmap(1:3,:,:), [], 4).');   %'
                    
                    %// We repeat each color 2 times (using 1:2 below) because each tail has 2 vertices
                    set(q.Tail, ...
                        'ColorBinding', 'interpolated', ...
                        'ColorData', reshape(cmap(1:2,:,:), [], 4).');

                end
                colorbar('eastoutside')
                legend('Demonstration dataset', 'Learned DS', 'Open loop trajectories', 'Location', 'best');

            end

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%   Step 5 (Evaluation): Compute Metrics and Visualize Velocities %%
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Compute Errors
            clc;
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
        
            if self.doPlots    
                % Compare Velocities from Demonstration vs DS
                h_vel = visualizeEstimatedVelocities(Data, self.dsControl);
            end

            %% %%%%%%%%%%%%%%%% Save ds to object %%%%%%%%%%%%%%%%%%%%%

            self.attractor = attractor;
            self.dsUnmodulated = @(x) ds_seds(x - attractor); 
            
            if self.doModulation  
                self.dsControl = @(x) self.modulatedDS(x);
            else
                self.dsControl = @(x) ds_seds(x - attractor);
            end


     
           disp('Finished learning with SEDS')


        end
        
        function setLinearDS(self, attractor)
            
            % Enforce column vector notation
            if size(attractor, 2) == 3
                attractor = attractor';
            end          

            % Condition for reasonable attractor
            if any(attractor > 2) ||  any(attractor < -2) 
                warning('Please provide an attractor inside the workspace !')
            else
                self.attractor = attractor;
                self.dsUnmodulated = @(x) -1*eye(3)*(x-attractor);

                if self.doModulation
                    self.dsControl = @(x) self.modulatedDS(x);
                else
                    self.dsControl = @(x) -1*eye(3)*(x - attractor);
                end
            end
        end
        
        % Set dsControl to use modulation
        function activateModulation(self)
            self.doModulation = 1;
            self.dsControl = @(x) self.modulatedDS(x);
        end
        
        % Set dsControl to dsUnmodulated 
        function deactivateModulation(self)
            self.doModulation = 0;
            self.dsControl = @(x) self.dsUnmodulated(x);
        end
       
        function xdot = modulatedDS(self, x)
        
            xdot_nominal = self.dsUnmodulated(x);
                   
            % Initialize modulation matrix and average obstacle velocity
            M_tot = eye(3);
            meanObstacleVelocity = zeros(3, 1);
        
            % Create the array of gamma values from the list world.listOfObstacles
            % for convenience
            kObst = length(self.myWorld.listOfObstacles);
            gamma = inf(1, kObst);
            for k = 1:kObst
                gamma(k) = max([self.myWorld.listOfObstacles(k).gammaDistance(x(1), x(2), x(3)) - self.myWorld.endEffectorRadius, 1]);
            end
            
            % Go over each obstacle, compute the modulation matrix and apply it to
            % the initial ds velocity xdot_modulated
            for k = 1:kObst
                
                % Compute distance function
                if kObst > 1
                    weight = (gamma - 1)./( (gamma - 1) + (gamma(k) - 1) );
                    weight = prod(weight(1:k-1))*prod(weight(k+1:end));
                else
                    weight = 1;
                end
        
                % Compute modulation matrix 
                gammaCorrected = abs(gamma(k))^(1/self.myWorld.listOfObstacles(k).rho);
                D = diag([1 - (weight / gammaCorrected), 1 + (weight / gammaCorrected), 1 + (weight / gammaCorrected)]);
            
                % Compute normal and tangent on ellipse
                normal = self.myWorld.listOfObstacles(k).gradientGamma(x(1), x(2), x(3));
                normal = normal / vecnorm(normal);
                tangent1 = [0; -normal(3); normal(2)];
                tangent2 = cross(normal, tangent1);
            
                E = [normal, tangent1, tangent2];
                M = E * D / E;
                M_tot = M * M_tot;
        
                % Weighted average of the obstacle velocities
                meanObstacleVelocity = meanObstacleVelocity + weight * self.myWorld.listOfObstacles(k).velocity;
            end    
        
            xdot = M_tot * (xdot_nominal - meanObstacleVelocity) + meanObstacleVelocity;
        end

    end
end
