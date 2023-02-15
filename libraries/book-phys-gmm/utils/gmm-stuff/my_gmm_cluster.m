function [Pk_x_max, labels] =  my_gmm_cluster(X, Priors, Mu, Sigma, type, softThresholds)
%MY_GMM_CLUSTER Computes the cluster labels for the data points given the GMM
%
%   input -----------------------------------------------------------------
%   
%       o X      : (N x M), a data set with M samples each being of dimension N.
%                           each column corresponds to a datapoint
%       o Priors : (1 x K), the set of priors (or mixing weights) for each
%                           k-th Gaussian component
%       o Mu     : (N x K), an NxK matrix corresponding to the centroids 
%                           mu = {mu^1,...mu^K}
%       o Sigma  : (N x N x K), an NxNxK matrix corresponding to the 
%                           Covariance matrices  Sigma = {Sigma^1,...,Sigma^K}
%       o type   : string ,{'hard', 'soft'} type of clustering
%
%       o softThresholds: (2 x 1), a vecor for the minimum and maximum of
%                           the threshold for soft clustering in that order
%
%   output ----------------------------------------------------------------
%
%       o labels   : (1 x M), a M dimensional vector with the label of the
%                             cluster for each datapoint
%                             - For hard clustering, the label is the 
%                             cluster number.
%                             - For soft clustering, the label is 0 for 
%                             data points which do not have high confidnce 
%                             in cluster assignment
%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Auxiliary Variables
[N, M] = size(X);
[~, K] = size(Mu);

% Initializing variables
Pk_x = zeros(K,M);
labels = zeros(1,M);
Pk_x_max = zeros(1,M);

% Find the a posteriori probability for each data point for each cluster
for ii = 1:K
    Px_k(ii,:) = my_gaussPDF(X,Mu(:,ii),Sigma(:,:,ii));
end

for ii=1:M
   Pk_x(:,ii) = (Priors'.*Px_k(:,ii))./(sum(Priors'.*Px_k(:,ii)) + eps);
end

for ii = 1:M
    
    switch type
        case 'hard'
            % Find the cluster with highest probability
            [Pk_x_max(ii) ,labels(ii)] = max(Pk_x(:,ii));
    
        case 'soft'
            % Find the cluster with highest probabilty. Unless, the highest
            % and another cluster are in the same range specified by
            % threshold
            softThresholdMin = softThresholds(1);
            softThresholdMax = softThresholds(2);

            [maxPk_x, maxPk_xIndex] = max(Pk_x(:,ii));
            flag = true;
            for kk = 1:K
                % Case 1
                if( maxPk_x < softThresholdMax && Pk_x(kk,ii) > softThresholdMin && kk~= maxPk_xIndex )              
                    flag = false;
                    break;
                end
                % Case 2
                if ( maxPk_x < softThresholdMin)
                    flag = false;
                    break;
                end
            end
                    
            if(flag)
                labels(ii) = maxPk_xIndex;                
            else
                labels(ii) = 0;
            end      
        otherwise
            fprintf('Invalid type for clustering\n');
            break;
            
    end
end

