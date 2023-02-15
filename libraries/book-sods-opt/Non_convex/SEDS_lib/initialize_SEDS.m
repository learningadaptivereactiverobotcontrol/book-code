function [Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,K,varargin)


%%
d = size(Data,1)/2; %number of dimensions

fprintf('\nStarting initialization ...\n')
%initializing with EM
Reapeat=1;
while Reapeat==1
    if exist('fitgmdist')==0
        options = statset('Display','off','MaxIter',5000);
        obj = gmdistribution.fit(transpose(Data),K,'Options',options,'Regularize',10^(-12),'CovType','diagonal','Replicates',100); 
        Priors_0=obj.PComponents;
    else
        options = statset('Display','off','MaxIter',5000);
        obj = fitgmdist(transpose(Data),K,'Options',options,'Regularize',10^(-12),'CovarianceType','diagonal','Replicates',100);
            Priors_0=obj.ComponentProportion;
    end
    Mu_0=transpose(obj.mu);
    for k=1:K
        if sum(Mu_0(1:d,k))==0
            Mu_0(1:d,k)=Mu_0(1:d,k)+0.0001;
        end
    end
    Reapeat=0;
    for k=1:K
        Sigma_0(:,:,k)=diag(obj.Sigma(:,:,k));
        if det(Sigma_0(:,:,k))==0
            Reapeat=1;
        end
    end
end
% [Priors_0, Mu_0, Sigma_0] = EM(Data);

if isempty(varargin)
    % deforming covariance fucntion such that they always satisfy the stability
    % conditions
    Sigma_tmp = Sigma_0;
    for k=1:K
        Sigma_0(:,:,k) = diag(diag(Sigma_tmp(:,:,k)));
        Sigma_0(1:d,d+1:end,k) = -diag(abs(diag(Sigma_tmp(d+1:2*d,1:d,k))));
        Sigma_0(d+1:end,1:d,k) = -diag(abs(diag(Sigma_tmp(d+1:2*d,1:d,k))));
    end
else
    options = varargin{1};
    for k=1:K
        Pxi(:,k) = Priors_0(k).*gaussPDF(Data, Mu_0(:,k), Sigma_0(:,:,k));
    end
    [~ , ind] = max(Pxi,[],2);
    options.perior_opt = 0;
    options.display = 0;
    options.normalization = 0;
    for k=1:K
        [~, Mu_0(:,k) Sigma_0(:,:,k)]=SEDS_Solver(Priors_0(k),Mu_0(:,k),Sigma_0(:,:,k),Data(:,ind==k),options);
    end
end
fprintf('Initialization finished successfully.\n')