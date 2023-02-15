function [x_dot] = lpv_ds(x, ds_gmm, A_g, b_g)

% Auxiliary Variables
[N,M] = size(x);
K = length(ds_gmm.Priors);

% Posterior Probabilities per local DS
beta_k_x = posterior_probs_gmm(x,ds_gmm,'norm');

% Output Velocity
x_dot = zeros(N,M);
for i = 1:size(x,2)
    % Estimate Global Dynamics component as LPV
    if size(b_g,2) > 1
        f_g = zeros(N,K);
        for k=1:K
            f_g(:,k) = beta_k_x(k,i) * (A_g(:,:,k)*x(:,i) + b_g(:,k));
        end
        f_g = sum(f_g,2);
    else
        % Estimate Global Dynamics component as Linear DS
        f_g = (A_g*x(:,i) + b_g);
    end
    x_dot(:,i) = f_g;          
end
end



