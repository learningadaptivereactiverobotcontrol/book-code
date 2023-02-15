function logLik = table_logLik(Y, lambda, type)
% Marginal log likelihood of data given hyper-parameters for Conjugate Prior
% for type='diag' uses Normal Gamma
% for type='full' uses Normal Inverse Wishart

[M,N]   = size(Y);

mu_0    = lambda.mu_0;
kappa_0 = lambda.kappa_0;
kappa_N = kappa_0 + N;

Ybar    = mean(Y,2);
Y_Ybar  = bsxfun(@minus, Y, Ybar);
Ybar_mu = Ybar - mu_0;

switch type
    case 'diag'                
        % Marginal log likelihood of NG distribution
        alpha_0 = lambda.alpha_0;
        beta_0 = lambda.beta_0;
        
        % Compute posterior update parameters Eq. 86-89
        % p(\mu, \lambda) = NG(\mu,\lambda | \mu_N, \kappa_n, \alpha_n,
        % \beta_n ) (page 8 . Conjugate Bayesian analysis of the Gaussian distribution 'Murphy')
        alpha_N = alpha_0 + N/2;
        beta_N  = beta_0  + 0.5*sum(Y_Ybar.^2,2) + (kappa_0*N/(2*(kappa_N))*(Ybar_mu).^2);               
        
        % Marginal Likelihood p(Y|\lambda) Eq. 95
        % (page 9 . Conjugate Bayesian analysis of the Gaussian distribution 'Murphy')
        % logLik = log(gamma_ratios) +  log(beta_ratios) +  log(kappa_ratios) +  log(constant)
        logLik = M*( gammaln(alpha_N) - gammaln(alpha_0) + ...
            alpha_0*log(beta_0) + 1/M*sum(-alpha_N*log(beta_N)) + ...
            0.5*(log(kappa_0) - log(kappa_N) - N*log(2*pi)));
        
    case 'full'        
        % Predicted log likelihood of NIW distribution Eq. 250-254
        % (page 20 . Conjugate Bayesian analysis of the Gaussian distribution 'Murphy')
        % p(\mu, \Sigma) = NIW(\mu,\Sigma | \mu_N, \kappa_n, \nu_n, \Lambda_n )

        nu_0     = lambda.nu_0;
        Lambda_0 = lambda.Lambda_0;
        nu_N     = nu_0 + N;
        S        = Y_Ybar*Y_Ybar';
        Lambda_N = Lambda_0 + S + (kappa_0*N/kappa_N)*(Ybar-mu_0)*(Ybar-mu_0)';
        
        % Marginal Likelihood p(Y|\lambda) Eq. 266
        % (page 21 . Conjugate Bayesian analysis of the Gaussian distribution 'Murphy')
        % logLik = log(gamma_ratios) +  (logdet_ratios) + log(kappa_ratios) + log (constant)
        logLik   =  logMvGamma( 0.5*nu_N, M) - logMvGamma( 0.5*nu_0, M) + ...
                    (nu_0/2)*logDet(Lambda_0) - (nu_N/2)*logDet(Lambda_N) + ...
                    0.5*M*(log(kappa_0)-log(kappa_N)) - 2*N*M*log(pi);
end






