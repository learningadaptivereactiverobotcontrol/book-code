function [gamma_k_x] = gamma_prob_fun(x,gmm, chosen_k)
% Truncated posterior probability
gamma_x = posterior_probs_gmm(x,gmm,'norm');
gamma_k_x = gamma_x(chosen_k, :);

% % Probability distribution of given data
% pdf_x =  ml_gmm_pdf(x, gmm.Priors, gmm.Mu, gmm.Sigma);
% pdf_x_uncert = pdf_x < 1e-5;
% 
% % If they are all below this threshold assign to a chosen k
% K = length(gmm.Priors);
% prob_replace = zeros(K,1);
% prob_replace(chosen_k,1) = 1; 
% for i=1:length(gamma_k_x)    
%     if pdf_x_uncert(i)
%         gamma_k_x(:,i) = prob_replace;
%     end
% end

end