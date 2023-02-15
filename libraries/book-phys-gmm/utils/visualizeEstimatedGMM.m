function [h_gmm, h_pdf] = visualizeEstimatedGMM(Xi_ref,  Priors, Mu, Sigma, est_labels, est_options)
M = size(Xi_ref,1);
if M == 2
    % Visualize Cluster Parameters Trajectory Data
    [h_gmm] = plotGMMParameters( Xi_ref, est_labels, Mu, Sigma);
    limits = axis;
    switch est_options.type
        case 0
            title('Physically-Consistent Non-Parametric Mixture Model','Interpreter','LaTex', 'FontSize',15);
        case 1
            title('Best fit GMM with EM-based BIC Model Selection','Interpreter','LaTex', 'FontSize',15);
        case 2
            title('Bayesian Non-Parametric Mixture Model (CRP-GMM)','Interpreter','LaTex', 'FontSize',15);
    end
    
    % Visualize PDF of fitted GMM
    ml_plot_gmm_pdf(Xi_ref, Priors, Mu, Sigma, limits);
    switch est_options.type
        case 0
            title('Physically-Consistent Gaussian Mixture Model','Interpreter','LaTex', 'FontSize',15);
        case 1
            title('Best fit GMM with EM-based BIC Model Selection','Interpreter','LaTex', 'FontSize',15);
        case 2
            title('Bayesian Non-Parametric Mixture Model (CRP-GMM)','Interpreter','LaTex', 'FontSize',15);
    end
    
elseif M == 3
    GMM = [];
    GMM.Priors = Priors; GMM.Mu = Mu; GMM.Sigma = Sigma;
    [h_gmm] = plot3DGMMParameters(Xi_ref, GMM, est_labels);
    switch est_options.type
        case 0
            title('Physically-Consistent Gaussian Mixture Model','Interpreter','LaTex', 'FontSize',15);
        case 1
            title('Best fit GMM with EM-based BIC Model Selection','Interpreter','LaTex', 'FontSize',15);
        case 2
            title('Bayesian Non-Parametric Mixture Model (CRP-GMM)','Interpreter','LaTex', 'FontSize',15);
    end
    view(-120, 30);
    axis equal
    h_pdf = [];
end


end