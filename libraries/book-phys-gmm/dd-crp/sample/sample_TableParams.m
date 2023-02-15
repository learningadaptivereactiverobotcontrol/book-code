function [Theta] = sample_TableParams(Y, Z_C, lambda, type)

% Compute lambda_N-parameters
new_lambdas = compute_lambdasN(Y, Z_C, lambda, type);

% New Cluster Means \mu = {mu_1, ... \mu_K}
Theta.Mu = new_lambdas.mu_N;

% New Cluster Covariance Martices \Sigma = {\Sigma_1, ... \Sigma_K}
[M, N] = size(Y); K = size(Theta.Mu, 2);
Sigma = zeros(M,M,K);

switch type
    case 'diag'
        % Compute precision values for each k-th table \lambda = {\lambda_1, ... \lambda_K}
        % from using NG seperately on each variable
        s2 = bsxfun(@rdivide,new_lambdas.beta_N,(new_lambdas.alpha_N.*new_lambdas.kappa_N));
        t = tinv(0.975, 2 * new_lambdas.alpha_N);
        Pr = bsxfun(@times,t./(2.*new_lambdas.alpha_N+1), sqrt(s2));
        for k=1:K
            Sigma(:,:,k) = diag(Pr(:,k));
        end
        
    case 'full'
        % Compute Covariance matrices for each k-th table \Sigma = {\Sigma_1, ... \Sigma_K}
        % from using NIW
        for k=1:K
            [sqrtSigma] = sample_iwishart(new_lambdas.Lambda_N(:,:,k),new_lambdas.nu_N(:,k));
            Sigma(:,:,k) = sqrtSigma'*sqrtSigma; % cholX'*cholX
        end
end
Theta.Sigma = Sigma;

end
