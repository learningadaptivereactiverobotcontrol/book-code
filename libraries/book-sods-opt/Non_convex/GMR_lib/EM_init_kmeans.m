function [Priors, Mu, Sigma] = EM_init_kmeans(Data, nbStates)
%
% This function initializes the parameters of a Gaussian Mixture Model 
% (GMM) by using k-means clustering algorithm.
%
% Inputs -----------------------------------------------------------------
%   o Data:     D x N array representing N datapoints of D dimensions.
%   o nbStates: Number K of GMM components.
% Outputs ----------------------------------------------------------------
%   o Priors:   1 x K array representing the prior probabilities of the
%               K GMM components.
%   o Mu:       D x K array representing the centers of the K GMM components.
%   o Sigma:    D x D x K array representing the covariance matrices of the 
%               K GMM components.
% Comments ---------------------------------------------------------------
%   o This function uses the 'kmeans' function from the MATLAB Statistics 
%     toolbox. If you are using a version of the 'netlab' toolbox that also
%     uses a function named 'kmeans', please rename the netlab function to
%     'kmeans_netlab.m' to avoid conflicts. 
%
% Copyright (c) 2006 Sylvain Calinon, LASA Lab, EPFL, CH-1015 Lausanne,
%               Switzerland, http://lasa.epfl.ch

[nbVar, nbData] = size(Data);

[Data_id, Centers] = kmeans(Data', nbStates,'MaxIter',5000);
% options = statset('Display','final');
% obj = gmdistribution.fit(Data',nbStates,'Options',options,'Regularize',0.001)
% obj=fitgmdist(Data,nbStates)
% idx = cluster(Data,obj);
% for i=1:nbStates
% if max(size(find(Data_id==i)))==1
%     idtmp=find(Data_id==i);
%     if idtmp==1
%         Data_id(2,1)=i;
%     else
%         Data_id(idtmp-1,1)=i;
%     end
% end
% end 
Mu = Centers'; 
for i=1:nbStates
  idtmp = find(Data_id==i);
  Priors(i) = length(idtmp);
  Sigma(:,:,i) = cov([Data(:,idtmp) Data(:,idtmp)]');
  %% Add a tiny variance to avoid numerical instability
  Sigma(:,:,i) = Sigma(:,:,i) + 1E-5.*diag(ones(nbVar,1));
end
Priors = Priors ./ sum(Priors);


