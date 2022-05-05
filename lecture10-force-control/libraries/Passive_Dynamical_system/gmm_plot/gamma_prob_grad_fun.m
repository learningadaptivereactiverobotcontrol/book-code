function [gamma_k_grad] = gamma_prob_grad_fun(x, gmm, chosen_k)

% Compute the posterior probs
pi_N = posterior_probs_gmm(x,gmm,'un-norm');
pi_N_k = pi_N(chosen_k, :);
pi_K_N = sum(pi_N,1);

% Scalar Factor
alpha_k_grad = pi_N_k./(pi_K_N.^2);


% Unpack gmm
Mu     = gmm.Mu;
Priors = gmm.Priors;
Sigma  = gmm.Sigma;
K      = length(Priors);

% Compute probabilities p(x^i|k)
for k=1:K
    Px_k(k,:) = ml_gaussPDF(x, Mu(:,k), Sigma(:,:,k)) + eps;
end

[N,M] = size(x);
gamma_k_grad = zeros(N,M);
for i=1:M
    grad_j = zeros(N,1);
    for j=1:K
        grad_j = grad_j + Priors(j)*Px_k(j,i)*( -inv(Sigma(:,:,chosen_k))*(x(:,i) - Mu(:,chosen_k)) + ...
            inv(Sigma(:,:,j))*(x(:,i)- Mu(:,j)));
    end
    gamma_k_grad(:,i) = alpha_k_grad(i)*grad_j;
end



end