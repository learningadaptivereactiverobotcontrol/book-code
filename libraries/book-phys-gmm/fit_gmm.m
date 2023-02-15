function [Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2018 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Nadia Figueroa                                                 % 
% email:   nadia.figueroafernandez@epfl.ch                                %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% This work was supported by the EU project Cogimon H2020-ICT-23-2014.    %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%                                                                         %
% If you use this code in your research please cite:                      %
% "A Physically-Consistent Bayesian Non-Parametric Mixture Model for      %
%   Dynamical System Learning."; N. Figueroa and A. Billard; CoRL 2018    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parse Options
est_type         = est_options.type;

do_plots         = est_options.do_plots;
[M,N]            = size(Xi_ref);

if est_type == 1
    max_gaussians    = est_options.maxK;
    if isempty(est_options.fixed_K)
        fixed_K        = 0;
    else
        fixed_K = est_options.fixed_K;
    end
end

if ~isempty(est_options.sub_sample)
    sub_sample       = est_options.sub_sample;
    Xi_ref     = Xi_ref(:,1:sub_sample:end);
    Xi_dot_ref = Xi_dot_ref(:,1:sub_sample:end);
end

if est_type ~= 1    
    if isempty(est_options.samplerIter)
        if est_type == 0
            samplerIter = 20;
        end
        if est_type == 2
            samplerIter = 200;
        end
    else
        samplerIter = est_options.samplerIter;
    end
    
    if ~isempty(est_options.l_sensitivity)
        l_sensitivity = est_options.l_sensitivity;
    else
        l_sensitivity = 2;
    end
end
switch est_type
    case 0
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%% Option1: Non-parametric Clustering with Pos-Vel-cos-sim prior %%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
        
        % Compute estimate of length-scale
        if est_options.estimate_l == 1
            [D, mode_hist_D, mean_D] = computePairwiseDistances(Xi_ref',1);
            if mode_hist_D == 0
                mode_hist_D = mean_D;
            end
            sigma = sqrt(mode_hist_D/l_sensitivity);
            l = 1/(2*sigma^2);
            close;
        else
            l = est_options.length_scale;
        end
        fprintf('Computed length-scale l=%2.4f \n',l);
        
        % Compute element-wise cosine similarities
        S = zeros(length(Xi_dot_ref),length(Xi_dot_ref));
        for i=1:length(Xi_dot_ref)
            for j=1:length(Xi_dot_ref)
                % *** Option 1 ***: Compute Velocity component using tan function
%                 % Normalize vectors
%                 xi_dot_i = Xi_dot_ref(:,i)/norm(Xi_dot_ref(:,i));
%                 xi_dot_j = Xi_dot_ref(:,j)/norm(Xi_dot_ref(:,j));                 
%                 % Compute Angle
%                 s_angle = atan2(xi_dot_i(2),xi_dot_i(1))-atan2(xi_dot_j(2),xi_dot_j(1));
%                 cos_angle = cos(s_angle);

                % *** Option 2 ***: Compute Velocity component using dot product
                cos_angle = (Xi_dot_ref(:,i)'*Xi_dot_ref(:,j))/(norm(Xi_dot_ref(:,i))*norm(Xi_dot_ref(:,j)));                                
                
                % Compute shifted cosine of angle
                if isnan(cos_angle)
                    cos_angle = 0;
                end
                s = 1 + cos_angle;
                
                % Compute Position component
                xi_i = Xi_ref(:,i);
                xi_j = Xi_ref(:,j);

                % Euclidean pairwise position-kernel
                p = exp(-l*norm(xi_i - xi_j));
                
                % Shifted Cosine Similarity of velocity vectors
                S(i,j) = p * s;
                
            end
        end
        
        % Plot Similarity matrix
        if do_plots
            if exist('h_sim','var');     delete(h_sim);    end;
            title_str = 'Physically-Consistent Similarity Confusion Matrix';
            h_sim = plotSimilarityConfMatrix(S, title_str);
            pause(0);
        end
        
        % Setting sampler/model options (i.e. hyper-parameters, alpha, Covariance matrix)
        Xi_ref_mean             = mean(Xi_ref,2);
        options                 = [];
        options.type            = 'full';                     % Type of Covariance Matrix: 'full' = NIW or 'Diag' = NIG
        options.T               = samplerIter;                % Sampler Iterations
%         options.alpha           = max(0.5,0.1*(randi(11)-2)); % Concentration parameter
        options.alpha           = 2; % maximum of similarity function
        
        % Standard Base Distribution Hyper-parameter setting
        if strcmp(options.type,'diag')
            lambda.alpha_0       = M;                               % G(sigma_k^-1|alpha_0,beta_0): (degrees of freedom)
            lambda.beta_0        = sum(diag(cov(Xi_ref')))/M;       % G(sigma_k^-1|alpha_0,beta_0): (precision)
        end
        if strcmp(options.type,'full')
            lambda.nu_0        = M;                                  % IW(Sigma_k|Lambda_0,nu_0): (degrees of freedom)
%             lambda.Lambda_0    = eye(M)*sum(diag(cov(Xi_ref')))/M;   % IW(Sigma_k|Lambda_0,nu_0): (Scale matrix)
            lambda.Lambda_0    = diag(diag(cov(Xi_ref')));       % IW(Sigma_k|Lambda_0,nu_0): (Scale matrix)
        end
        lambda.mu_0             = Xi_ref_mean;                  % hyper for N(mu_k|mu_0,kappa_0)
        lambda.kappa_0          = 1;                            % hyper for N(mu_k|mu_0,kappa_0)       
        
        % Run Collapsed Gibbs Sampler
        options.lambda    = lambda;
        options.verbose   = 1;
        [Psi Psi_Stats]   = run_ddCRP_sampler(Xi_ref, S, options);
        est_labels        = Psi.Z_C';
        
        % Extract Learnt cluster parameters
        unique_labels = unique(est_labels);                                
        est_K      = length(unique_labels);
        Priors     = zeros(1, est_K);
        singletons = zeros(1, est_K);        
        for k=1:est_K
            assigned_k = sum(est_labels==unique_labels(k));
            Priors(k) = assigned_k/N;
            singletons(k) = assigned_k < 2;
        end
        Mu     = Psi.Theta.Mu;
        Sigma  = Psi.Theta.Sigma;
            
        if any(singletons)
            [~, est_labels] =  my_gmm_cluster(Xi_ref, Priors, Mu, Sigma, 'hard', []);
            unique_labels = unique(est_labels);                                
            est_K         = length(unique_labels);
            singleton_idx = find(singletons == 1);
            Mu(:,singleton_idx) = [];
            Sigma(:,:,singleton_idx) = [];
            Priors  = [];
            for k=1:est_K
                assigned_k = sum(est_labels==unique_labels(k));
                Priors(k) = assigned_k/N;               
            end
        end
        
        %% Re-estimate GMM parameters, needed for >2D data
        if M > 2
            Mu_k = Mu;  Sigma_k = Sigma;
            for k=1:length(unique_labels)
                cluster_points = Xi_ref(:,est_labels == unique_labels(k));
                if ~isempty(cluster_points)
                    [ V_k, L_k, Mu_k(:,k) ] = my_pca( cluster_points );
                    Sigma_k(:,:,k) = V_k*L_k*V_k';
                end
            end
            rel_dilation_fact = 0.15;
            Sigma_k = adjust_Covariances(Priors, Sigma_k, 1, rel_dilation_fact);
            Mu    = Mu_k;
            Sigma = Sigma_k;
        end        
        
        if do_plots
            if exist('h1b','var') && isvalid(h1b), delete(h1b);end
            stats_options = [];
            stats_options.dataset      = 'Sampler Stats';
            stats_options.true_labels  = [];
            stats_options.Psi          = Psi;
            [ h1b ] = plotSamplerStats( Psi_Stats, stats_options );
        end        
        
    case 1
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%% Option2: Cluster Trajectories with GMM-EM + BIC Model Selection %%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if fixed_K == 0
            em_type = 'matlab'; repetitions = 10;
            [bic_scores, k] = fit_gmm_bic([Xi_ref],max_gaussians, repetitions, em_type, do_plots);
        else
            k = fixed_K;
        end
        % Train GMM with Optimal k
        warning('off', 'all'); % there are a lot of really annoying warnings when fitting GMMs
        %fit a GMM to our data
        GMM_full = fitgmdist([Xi_ref]', k, 'Start', 'plus', 'CovarianceType','full', 'Regularize', .000001, 'Replicates', 10); 
        warning('on', 'all');
        
        % Extract Model Parameters
        Priors = GMM_full.ComponentProportion;
        Mu = transpose(GMM_full.mu);
        Sigma = GMM_full.Sigma;                

    case 2
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%% Option3: Cluster Trajectories with Chinese Restaurant Process MM sampler (CRP-GMM) %%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % CRP-GMM (Frank-Wood's implementation) -- faster
        [class_id, mean_record, covariance_record, K_record, lP_record, alpha_record] = sampler(Xi_ref, samplerIter);
        [val , Maxiter]  = max(lP_record);
        est_labels       = class_id(:,Maxiter);
        
        % Visualization and plotting options
        if do_plots
            figure('Color',[1 1 1])
            subplot(2,1,1)
            semilogx(1:samplerIter, lP_record'); hold on;
            semilogx(Maxiter,lP_record(Maxiter),'ko','MarkerSize',10);
            grid on
            xlabel('Gibbs Iteration','Interpreter','LaTex','Fontsize',20); ylabel('LogPr','Interpreter','LaTex','Fontsize',20)
            xlim([1 samplerIter])
            legend({'$p(Z|Y, \alpha, \lambda)$'},'Interpreter','LaTex','Fontsize',14)
            title(sprintf('CRP-GMM Sampling results, optimal K=%d at iter=%d', length(unique(est_labels)), Maxiter), 'Interpreter','LaTex','Fontsize',20)            
            subplot(2,1,2)
            stairs(K_record, 'LineWidth',2);
            set(gca, 'XScale', 'log')
            xlim([1 samplerIter])
            xlabel('Gibbs Iteration','Interpreter','LaTex','Fontsize',20); ylabel('$\Psi$ = Estimated K','Interpreter','LaTex','Fontsize',20);
        end

        % Extract Learnt cluster parameters
        unique_labels = unique(est_labels);                              
        est_K         = length(unique_labels);
        Priors        = zeros(1, est_K);
        singletons    = zeros(1, est_K);        
        for k=1:est_K
            assigned_k = sum(est_labels==unique_labels(k));
            Priors(k) = assigned_k/N;
            singletons(k) = assigned_k < 2;
        end
        Mu    = mean_record {Maxiter};
        Sigma = covariance_record{Maxiter};
        
        % Remove Singleton Clusters
        if any(singletons)
            [~, est_labels] =  my_gmm_cluster(Xi_ref, Priors, Mu, Sigma, 'hard', []);
            unique_labels = unique(est_labels);                                
            est_K         = length(unique_labels);
            Mu    = Mu(:,unique_labels);
            Sigma = Sigma(:,:,unique_labels);
            Priors  = [];
            for k=1:est_K
                assigned_k = sum(est_labels==unique_labels(k));
                Priors(k) = assigned_k/N;               
            end
        end
        

end


end

