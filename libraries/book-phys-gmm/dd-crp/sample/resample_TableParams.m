function [Theta] = resample_TableParams(Y, Z_C, lambda, type)

    % Updating lambda-parameters
    new_lambdas = update_lambdas(Y, Z_C, lambda, type);
    
    % New Cluster Means \mu = {mu_1, ... \mu_K}
    Mu = new_lambdas.mu_N;
    
    switch type
        case 'diag'
            % Computing new precision values \lambda = {\lambda_1, ... \lambda_K}
            s2 = bsxfun(@rdivide,new_lambdas.beta_N,(new_lambdas.alpha_N.*new_lambdas.kappa_N));
            t = tinv(0.975, 2 * new_lambdas.alpha_N);
            Pr = bsxfun(@times,t./(2.*new_lambdas.alpha_N+1),sqrt(s2));
            
        case 'full'     
            % Computing new cluster Covariance matrices \Sigma = {\Sigma_1, ... \Sigma_K}
            
    end
    
    Theta.Mu = Mu;
    Theta.Pr = Pr;
    [M, N] = size(Y); K = size(Theta.Mu, 2);
    Sigma = zeros(M,M,K);
    for k=1:K
            Sigma(:,:,k) = diag(Pr(:,k));
    end
    Theta.Sigma = Sigma;    
end
