function [Sigma] = adjust_Covariances(Priors, Sigma, tot_scale_fact, rel_scale_fact)

% Check Relative Covariance Matrix Eigenvalues
est_K = size(Sigma,3);
dim = size(Sigma(:,:,1),1);
Vs = zeros(dim,dim,est_K);
Ls = zeros(dim,dim,est_K);
p1_eig = []; p2_eig = []; p3_eig = [];

baseline_prior = (0.5/length(Priors));

for k=1:est_K                    
    [Vs(:,:,k), Ls(:,:,k)] = eig(Sigma(:,:,k));           
    if ~issorted(diag(Ls(:,:,k)))
        [L_,ids] = sort(diag(Ls(:,:,k)));
        Ls(:,:,k) = diag(L_);
        Vs(:,:,k) = Vs(:,ids,k);
    end
    if Priors(k) > baseline_prior
        Ls(:,:,k) = tot_scale_fact*Ls(:,:,k);
    end
    lambda_1 = Ls(1,1,k);
    lambda_2 = Ls(2,2,k);
    p1_eig = [p1_eig lambda_1];
    p2_eig = [p2_eig lambda_2];
    if dim   == 3
        lambda_3 = Ls(3,3,k);
        p3_eig = [p3_eig lambda_3];
    end
    Sigma(:,:,k)    = Vs(:,:,k) * Ls(:,:,k) * Vs(:,:,k)';
end

% Scale Sigma's to increase the influence of the local linear dynamics
if dim == 2
    cov_ratios = p1_eig./p2_eig;
    for k=1:est_K
        if (cov_ratios(k) < rel_scale_fact)
            lambda_1 = p1_eig(k); lambda_2 = p2_eig(k);
            lambda_1_ = lambda_1 + lambda_2*(rel_scale_fact-cov_ratios(k));
            Sigma(:,:,k)    = Vs(:,:,k) * diag([ lambda_1_ ; lambda_2 ]) * Vs(:,:,k)';
        end
        
    end
elseif dim == 3
        cov_ratios = p2_eig./p3_eig;
        for k=1:est_K
            if (cov_ratios(k) < rel_scale_fact)
                lambda_1 = p1_eig(k);  lambda_2 = p2_eig(k); lambda_3 = p3_eig(k);                
                lambda_2_ = lambda_2 + lambda_3*(rel_scale_fact-cov_ratios(k));
                lambda_1_ = lambda_1 + lambda_3*(rel_scale_fact-cov_ratios(k));
                Sigma(:,:,k)    = Vs(:,:,k) * diag([ lambda_1_ ; lambda_2_; lambda_3 ]) * Vs(:,:,k)';
            end

        end 
end

% Re-group Sigma
ds_gmm.Sigma = Sigma;
    
end