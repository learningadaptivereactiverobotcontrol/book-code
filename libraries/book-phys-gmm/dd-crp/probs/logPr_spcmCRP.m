function [post_LogProb data_LogLik] = logPr_spcmCRP(Y, delta, Psi)
% Current Markov State
[~,N] = size(Y);
C = Psi.C;
Z_C = Psi.Z_C;
clust_ids = unique(Z_C);

% Hyperparameters
alpha  = Psi.alpha;
lambda = Psi.lambda;
type   = Psi.type;

%%% Compute Prior Probability on Partition %%%
prior_LogLik = 0;
for i = 1:length(C)
    if i==C(i)
        prior_LogLik = prior_LogLik + log(alpha./(alpha+N));
    else
        prior_LogLik = prior_LogLik + log(delta{i}(C(i))./(alpha+sum(delta{i})));
    end
end

%%% Compute Likelihood of Partition %%%
data_LogLik = 0;
for i = 1:length(clust_ids)
    k = clust_ids(i);
    if ~(sum(Z_C==k)==1 && isempty(Z_C==k))
        data_LogLik = data_LogLik + table_logLik(Y(:,Z_C==k),lambda, type);
    end
end

%%% Posterior Log Prob of partitions given observations  %%%
post_LogProb = prior_LogLik + data_LogLik;
