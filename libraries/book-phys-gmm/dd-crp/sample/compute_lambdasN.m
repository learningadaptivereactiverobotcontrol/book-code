function [new_lambdas] = compute_lambdasN(Y, Z_C, lambda, type)

% Extract # Clusters and Auxiliary variables
[M, ~] = size(Y);
K = length(unique(Z_C));

% Normal Hyperparameters
mu_0    = lambda.mu_0;
kappa_0 = lambda.kappa_0;

% Compute Parameters for each K tables
mu_N    = zeros(M, K);
kappa_N = zeros(1, K);

switch type
    case 'diag'
        alpha_N = zeros(1, K);
        beta_N  = zeros(M, K);
    case 'full'
        nu_N      = zeros(1, K);
        Lambda_N  = zeros(M, M, K);
end

for k=1:K
    % Auxiliary Variables per Tables
    N       = size(Y(:,Z_C == k),2);
    Ybar    = mean(Y(:,Z_C == k),2);
    YbarN   = Ybar*N;
    Y_Ybar = bsxfun(@minus, Y(:,Z_C == k), Ybar);
    Y_YbarN = bsxfun(@minus, Y(:,Z_C == k), YbarN);
    Ybar_mu = Ybar - mu_0;
    
    % Compute Mean Parameters K-th table
    mu_N(:,k) = (kappa_0.*mu_0 + YbarN)./(kappa_0 + N);
    
    % Compute Kappa Parameters K-th table
    kappa_N(:,k) = kappa_0 + N;
        
    switch type
        case 'diag'
            % Gamma Hyperparameters
            alpha_0 = lambda.alpha_0;
            beta_0  = lambda.beta_0;
            
            % Compute posterior update parameters Eq. 86-89
            % p(\mu, \lambda) = NG(\mu,\lambda | \mu_N, \kappa_n, \alpha_n,
            % \beta_n ) (page 8 . Conjugate Bayesian analysis of the Gaussian distribution 'Murphy')
            alpha_N(:,k) = alpha_0 + N/2; 
            beta_N(:,k)  = beta_0  + 0.5*sum(Y_YbarN.^2,2) + (kappa_0*N/(2*(kappa_0 + N))*(Ybar_mu).^2);
            
        case 'full'
            
            % Predicted log likelihood of NIW distribution Eq. 250-254
            % p(\mu, \Sigma) = NIW(\mu,\Sigma | \mu_N, \kappa_n, \nu_n, \Lambda_n )
            
            nu_0     = lambda.nu_0;
            Lambda_0 = lambda.Lambda_0;
            nu_N(:,k)     = nu_0 + N;
            S        = Y_Ybar*Y_Ybar';
            Lambda_N(:,:,k) = Lambda_0 + S + (kappa_0*N/kappa_N(:,k))*(Ybar-mu_0)*(Ybar-mu_0)';
            
    end
    
    
end

new_lambdas.mu_N    = mu_N;
new_lambdas.kappa_N = kappa_N;

switch type
    case 'diag'
        new_lambdas.alpha_N   = alpha_N;
        new_lambdas.beta_N    = beta_N;
    case 'full'
        new_lambdas.nu_N      = nu_N;
        new_lambdas.Lambda_N  = Lambda_N;
end

end