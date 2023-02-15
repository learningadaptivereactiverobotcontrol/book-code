%% Check components of GMM-2 vs GMM-1 -- OLD SCRIPT
est_K_1 = length(ds_gmm_1.Priors);
clc;
for k=1:est_K_2
   fprintf('********** Evaluating component k=%d from GMM bathc b+1 **********\n',k) 
   D_k = Xi_ref_2(:,est_labels_2==k);
   d = size(D_k,1);  n = size(D_k,2);  
   nu = d*(d+1)/2; 
   
   % Variables of j-th component
   Mu_k    = ds_gmm_2.Mu(:,k);
   Sigma_k = ds_gmm_2.Sigma(:,:,k);

   for j=1:est_K_1       
       % Variables of j-th component
       Mu_j    = ds_gmm_1.Mu(:,j); 
       Sigma_j = ds_gmm_1.Sigma(:,:,j);
       
       % Test KL-Divergence
       KL_div_1 = 0.5 * (log(det(Sigma_k)/det(Sigma_j)) - d + trace(inv(Sigma_k)*Sigma_j) + ...
                      (Mu_k-Mu_j)'*inv(Sigma_k)*(Mu_k-Mu_j));
       KL_div_2 = 0.5 * (log(det(Sigma_j)/det(Sigma_k)) - d + trace(inv(Sigma_j)*Sigma_k) + ...
                      (Mu_j-Mu_k)'*inv(Sigma_j)*(Mu_j-Mu_k));
       
       if KL_div_1 < 1 || KL_div_2 < 1
            fprintf(2,'KL-divergence(j,k)=%2.2f and KL-divergence(k,j)=%2.2f\n',KL_div_1, KL_div_2);
            fprintf(2,'Kb(%d) and Kb=1(%d) are practically the same! \n',j, k);
       else
            fprintf('KL-divergence(j,k)=%2.2f and KL-divergence(k,j)=%2.2f\n',KL_div_1, KL_div_2);
       end
       % Cholesky decomposition of Sigma_j
       [L_0,~] = chol(Sigma_j,'lower');
       % Transformed data      
       y =  L_0\D_k;                  
       S_y = cov(y');  
       
       % Calculate W statistic
       W = ((1/d)*trace((S_y - eye(d)).^2)) - ((d/n)*(1/d * trace(S_y)).^2) + d/n;
       W_stat = (n*W*d)/2;        
       chi_value = chi2inv(0.9999,nu);       
       
       fprintf ('W-statistic %2.4f vs critical chi-value %2.4f\n', W_stat, chi_value);
       % Reject if W_test exceeds the upper alpha critical value
       if W_stat < chi_value
           fprintf('...K_b(%d) with K_b+1(%d) Passed the Covariance Test\n',j,k);
            x_bar = mean(D_k,2); 
%             S = cov(D_k');
            S = Sigma_j;
            T_squared = n * (x_bar - Mu_j)'*inv(S)*(x_bar - Mu_j);
            T_stat = ((n-d) / (d*(n-1)))*T_squared;
            f_value = finv(0.999,d,n-d);
            fprintf ('...T-statistic %2.4f vs critical f-value %2.4f\n', T_stat, f_value)
            if T_stat < f_value
                fprintf (2,'****Should merge/reject the pair components K_b(%d) with K_b+1(%d) with!!\n',j,k);
            end
       end
   end
end


%% Update GMM with data from second model
[Pk_x_max, est_labels] =  my_gmm_cluster(Xi_ref_2, ds_gmm_1.Priors, ds_gmm_1.Mu, ds_gmm_1.Sigma, 'hard', [ ]);
K_b = length(ds_gmm_1.Priors);
est_labels_conf = est_labels;
est_labels_conf(Pk_x_max < 1/K_b) = K_b + 1;

clust_colors = lines(K_b);
clust_colors = [clust_colors; 0 0 0];

figure('Color',[1 1 1])
for jj=1:K_b+1    
    scatter(Xi_ref_2(1,est_labels_conf==jj),Xi_ref_2(2,est_labels_conf==jj), 50, clust_colors(jj,:), 'filled'); hold on;
    if jj<(K_b+1)
        plotGMM(ds_gmm_1.Mu(:,jj), ds_gmm_1.Sigma(:,:,jj), clust_colors(jj,:), 1);
        alpha(.5)
    end    
end
grid on;