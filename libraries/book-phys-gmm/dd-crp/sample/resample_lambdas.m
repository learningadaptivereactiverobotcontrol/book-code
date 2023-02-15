function [new_lambdas] = update_lambdas(Y, Z_C, lambda, type)

    K = max(Z_C);
    Z = bsxfun(@eq,Z_C,1:K);
    
    Nks = sum(Z);
    YbarN = Y*Z; % Ybar*N
    Ybar = bsxfun(@rdivide,YbarN,Nks);
    
    % Updating Mean Parameters
    new_lambdas.mu_n = bsxfun(@rdivide,lambda.kappa_0.*lambda.mu_0 + YbarN, lambda.kappa_0+Nks);
    new_lambdas.kappa_n = Nks + lambda.kappa_0;
    
    size(mu_n)
    
    switch type
        case 'diag'
            % Update Precision Parameters (NG)
            new_lambdas.alpha_n = lambda.alpha_0 + Nks./2;
            new_lambdas.beta_n  = lambda.beta_0 + 0.5 * ((Y-YbarN(:,Z_C)).^2)*Z + bsxfun(@rdivide,lambda.kappa_0.* bsxfun(@times,Nks, (Ybar-lambda.mu_0).^2),2.*(lambda.kappa_0+Nks));
            
        case 'full'
            % Update Covariance Parameters (NIW)
    end
    
   
end