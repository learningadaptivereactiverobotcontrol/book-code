function [ds_gmm, A_g, b_g, P_est] = learn_LPVDS(gmm_type,constr_type, init_cvx, Data, att_g, limits, varargin)

Xi_ref     = Data(1:2,:);
Xi_dot_ref = Data(3:4,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%              Step 1: Fit GMM to Trajectory Data        %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% GMM Estimation Algorithm %%%%
% 0: Physically-Consistent Non-Parametric (Collapsed Gibbs Sampler)
% 1: GMM-EM Model Selection via BIC
% 2: GMM via Competitive-EM - rarely works..
% 3: CRP-GMM via Collapsed Gibbs Sampler

est_options = [];
est_options.type        = gmm_type;   % GMM Estimation Alorithm Type    
est_options.maxK        = 15;  % Maximum Gaussians for Type 1/2
est_options.do_plots    = 0;   % Plot Estimation Statistics
est_options.adjusts_C   = 1;   % Adjust Sigmas
est_options.fixed_K     = 5;   % Fix K and estimate with EM
est_options.exp_scaling = 1;    % Scaling for the similarity to improve locality

% Discover Local Models
sample = 2;
[Priors0, Mu0, Sigma0] = discover_local_models(Xi_ref(:,1:sample:end), Xi_dot_ref(:,1:sample:end), est_options);

% Extract Cluster Labels
est_K      = length(Priors0); 
Priors = Priors0; Mu = Mu0; Sigma = Sigma0;
[~, est_labels] =  my_gmm_cluster(Xi_ref, Priors, Mu, Sigma, 'hard', []);

% Visualize Cluster Parameters on Manifold Data
% plotGMMParameters( Xi_ref, est_labels, Mu, Sigma);
% limits_ = limits + [-0.015 0.015 -0.015 0.015];
% axis(limits_)
% switch est_options.type   
%     case 0
%         title('Physically-Consistent Non-Parametric Mixture Model','Interpreter','LaTex', 'FontSize',15);
%     case 1        
%         title('Best fit GMM with EM-based BIC Model Selection','Interpreter','LaTex', 'FontSize',15);
%     case 3
%         title('Bayesian Non-Parametric Mixture Model','Interpreter','LaTex', 'FontSize',15);
% end

%%% Visualize GMM pdf from learnt parameters 
% ml_plot_gmm_pdf(Xi_ref, Priors, Mu, Sigma,limits)

clear ds_gmm; ds_gmm.Mu = Mu; ds_gmm.Sigma = Sigma; 
ds_gmm.Priors = Priors; 
% Adjust Covariance Matrices
if est_options.adjusts_C  == 1
    tot_scale_fact = 1; rel_scale_fact = 0.15;
    Sigma = adjust_Covariances(Sigma0, tot_scale_fact, rel_scale_fact);
    ds_gmm.Sigma;
end    

% Visualize Cluster Parameters on Manifold Data
plotGMMParameters( Xi_ref, est_labels, Mu, Sigma);
limits_ = limits + [-0.015 0.015 -0.015 0.015];
axis(limits_)
switch est_options.type
    case 0
        title('Physically-Consistent Non-Parametric Mixture Model','Interpreter','LaTex', 'FontSize',15);
    case 1
        title('Best fit GMM with EM-based BIC Model Selection','Interpreter','LaTex', 'FontSize',15);
    case 3
        title('Bayesian Non-Parametric Mixture Model','Interpreter','LaTex', 'FontSize',15);
end

ml_plot_gmm_pdf(Xi_ref, Priors, Mu, Sigma, limits)


if constr_type == 0 || constr_type == 1
    P_opt = [];
else
    if nargin == 7
        P_opt = varargin{1};
    else
        [Vxf] = learn_wsaqf(Data, att_g);
        P_opt = Vxf.P(:,:,1);
    end
end

%%%%%%%%  LPV system sum_{k=1}^{K}\gamma_k(xi)(A_kxi + b_k) %%%%%%%%            
[A_g, b_g, P_est] = optimize_lpv_ds_from_data(Data, att_g, constr_type, ds_gmm, P_opt, init_cvx);



end