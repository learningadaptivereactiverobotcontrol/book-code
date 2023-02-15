function [bic_scores, k] = fit_gmm_bic(X,max_gaussians, repetitions, em_type, do_plot)
% Fit GMM with BIC Curve
fprintf('K = ');
dimz = size(X,1); 
bic_scores = [];
repetitions = 10;
for j = 1:max_gaussians
    fprintf('%d ',j);
    k_tmp = j;
    
    switch em_type
        case 'matlab'
            warning('off', 'all'); % there are a lot of really annoying warnings when fitting GMMs
            GMM_full_tmp = fitgmdist(X', k_tmp, 'Start', 'plus', 'CovarianceType','full', 'Regularize', .000001, 'Replicates', repetitions); %fit a GMM to our data
            warning('on', 'all');
            Priors_tmp = GMM_full_tmp.ComponentProportion;
            Mu_tmp = transpose(GMM_full_tmp.mu(:, 1:dimz));
            Sigma_tmp = GMM_full_tmp.Sigma(1:dimz, 1:dimz, :);
                        
            % We compute the likelihood of this GMM (with this number of modes k)
            bic_tmp = GMM_BIC(X, ones(size(X, 2), 1), Priors_tmp, Mu_tmp, Sigma_tmp, 'full');
            
        case 'nadia'
            rng(randi(10));
            cov_type = 'full';  Max_iter = 500;
            bic_min = inf;
            for r=1:repetitions
                [Priors0, Mu0, ~, Sigma0] = my_gmmInit(X, k_tmp, cov_type);
                [Priors, Mu, Sigma, ~]    = my_gmmEM(X, k_tmp, cov_type, Priors0, Mu0, Sigma0, Max_iter);
                bic_tmp_ = GMM_BIC(X, ones(size(X, 2), 1), Priors, Mu, Sigma, 'full');
                if bic_tmp_ <= bic_min
                    bic_min = bic_tmp_;
                end                
            end
            bic_tmp = bic_min;
    end    
    bic_scores = [bic_scores bic_tmp];
end
fprintf('\n ');

% % Compute Diff of BIC Scores
bic_scores_diff  = [0 diff(bic_scores)];
bic_scores_diff2 = [0 diff(bic_scores_diff)];

% Find optimal value on RSS curve
[~, opt_Ks_BIC_line] = ml_curve_opt(bic_scores,'line');
opt_BIC_vals_line    = bic_scores(opt_Ks_BIC_line);

% Other options with the 'derivatives' approach
[~, opt_Ks_BIC_der] = ml_curve_opt(bic_scores,'derivatives');
opt_BIC_vals_der    = bic_scores(opt_Ks_BIC_der);

if opt_Ks_BIC_line < opt_Ks_BIC_der(1)
    chosen_k = opt_Ks_BIC_der(1);
else
    chosen_k = opt_Ks_BIC_line;
end


% Select the point before this
k = chosen_k;
best_BIC = bic_scores(k);

% Plot Results
if do_plot
    figure('Color', [1 1 1])
    subplot(2,1,1)
    plot(1:length(bic_scores), bic_scores, '-*', 'Color', [rand rand rand]); hold on;
    scatter(k, best_BIC, 100, [0 0 0]); hold on;
    scatter(opt_Ks_BIC_line, opt_BIC_vals_line, 75, [1 0 0]); hold on;
    scatter(opt_Ks_BIC_der,opt_BIC_vals_der, 75, [0 1 0]); hold on;    
    grid on;
    title('BIC Score for GMM fit ','Interpreter','LaTex');
    xlabel('Gaussian functions $K$','Interpreter','LaTex');
    legend('BIC','Optimal K')
    
    subplot(2,1,2)
    plot(1:length(bic_scores), bic_scores_diff, '-*', 'Color', [rand rand rand]); hold on;
    plot(1:length(bic_scores), bic_scores_diff2, '-*', 'Color', [rand rand rand]); hold on;
    grid on;
    title('BIC Score for GMM fit ','Interpreter','LaTex');
    xlabel('Gaussian functions $K$','Interpreter','LaTex');
    
    legend('diff(BIC)','diff2(BIC)')
end
end

