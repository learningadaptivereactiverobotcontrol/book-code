function [Pk_x] = posterior_probs_gmm(x, gmm, type)

[N,M] = size(x);
% Unpack gmm
Mu     = gmm.Mu;
Priors = gmm.Priors;
Sigma  = gmm.Sigma;
K      = length(Priors);

% Compute mixing weights for multiple dynamics
Px_k = zeros(K,M);

% Compute probabilities p(x^i|k)
for k=1:K
    Px_k(k,:) = ml_gaussPDF(x, Mu(:,k), Sigma(:,:,k)) + eps;
end
%%% Compute posterior probabilities p(k|x) -- FAST WAY --- %%%
alpha_Px_k = repmat(Priors',[1 M]).*Px_k;

switch type
    case 'norm'
        Pk_x = alpha_Px_k ./ repmat(sum(alpha_Px_k,1),[K 1]);
    case 'un-norm'
        Pk_x = alpha_Px_k ;        
end