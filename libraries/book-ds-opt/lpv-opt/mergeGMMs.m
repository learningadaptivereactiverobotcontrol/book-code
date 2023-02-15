function [new_ds_gmm] = mergeGMMs(ds_gmm_1, ds_gmm_2)

% Merging two GMM models
K_merge      = length(ds_gmm_1.Priors) + length(ds_gmm_2.Priors);
Mu_merge     = zeros(2,K_merge);
Sigma_merge  = zeros(2,2,K_merge);
Priors_merge = zeros(1,K_merge);

% First model
Mu_merge(:,1:length(ds_gmm_1.Priors))      =  ds_gmm_1.Mu;
Sigma_merge(:,:,1:length(ds_gmm_1.Priors)) =  ds_gmm_1.Sigma;
Priors_merge(1,1:length(ds_gmm_1.Priors))  = (ds_gmm_1.Priors*ds_gmm_1.N)./(ds_gmm_1.N + ds_gmm_2.N);

% Second model
Mu_merge(:,length(ds_gmm_1.Priors)+1:end)      =  ds_gmm_2.Mu;
Sigma_merge(:,:,length(ds_gmm_1.Priors)+1:end) =  ds_gmm_2.Sigma;
Priors_merge(1,length(ds_gmm_1.Priors)+1:end)  = (ds_gmm_2.Priors*ds_gmm_2.N)./(ds_gmm_1.N + ds_gmm_2.N);

% Create new GMM structure
new_ds_gmm = [];
new_ds_gmm.Mu = Mu_merge; new_ds_gmm.Sigma = Sigma_merge; new_ds_gmm.Priors = Priors_merge;
new_ds_gmm.N = ds_gmm_1.N + ds_gmm_2.N;


end