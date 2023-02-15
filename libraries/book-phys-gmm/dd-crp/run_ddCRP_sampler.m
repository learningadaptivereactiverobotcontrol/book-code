function [Psi, Psi_Stats] = run_ddCRP_sampler(Y,S, options)
% Distance Depenendent Chinese Restaurant Process Mixture Model.
% **Inputs**
%          Y: projected M-dimensional points  Y (y1,...,yN) where N = dim(S),
%          S: Similarity Matrix where s_ij=1 is full similarity and
%          s_ij=0 no similarity between observations
%          
% **Outputs**
%          Psi (MAP Markov Chain State)
%          Psi.LogProb:
%          Psi.Z_C:
%          Psi.clust_params:
%          Psi.iter:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2018 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Nadia Figueroa                                                 % 
% email:   nadia.figueroafernandez@epfl.ch                                %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% This work was supported by the EU project Cogimon H2020-ICT-23-2014.    %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%                                                                         %
% If you use this code in your research please cite:                      %
% "A Physically-Consistent Bayesian Non-Parametric Mixture Model for      %
%   Dynamical System Learning."; N. Figueroa and A. Billard; CoRL 2018    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Parse Sampler Options and Set Aurxiliary Variables            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Data Dimensionality
% (M = reduced spectral space, N = # of observations)
[M, N] = size(Y);

%%%% Default Hyperparameters %%%%
T                = 100;
alpha            = 1;
type             = 'full';
lambda.mu_0      = 0;
lambda.kappa_0   = 1;
lambda.nu_0      = M;
lambda.Lambda_0  = eye(M)*M*0.5;

C = 1:N;
%%%% Parse Sampler Options %%%%
if nargin > 2
    if isfield(options, 'T');      T = options.T;end
    if isfield(options, 'alpha');  alpha = options.alpha;end          
    if isfield(options, 'type');   type = options.type;end    
    if isfield(options, 'lambda'); clear lambda; lambda = options.lambda;end
    if isfield(options, 'init_clust'); clear C; C = options.init_clust;end
end

%%% Initialize Stats Variabes  %%%
Psi_Stats.CompTimes    = zeros(1,T);
Psi_Stats.PostLogProbs = zeros(1,T);
Psi_Stats.LogLiks      = zeros(1,T);
Psi_Stats.TotalClust   = zeros(1,T);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Define Initial Markov Chain State Psi^{t-1}               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Augment Similarity Matrix with alpha on diagonal %%%
% S = S + eye(N)*(alpha-1);
S = S + eye(N)*(alpha);
S_alpha = num2cell(S,2);

%%% Compute Initial Customer/Table Assignments and Likelihoods %%%
table_members = cell(N,1);
Z_C   = extract_TableIds(C);
K = max(Z_C);
for k = 1:K
    table_members{k} = find(Z_C==k);    
    table_logLiks(k) = table_logLik(Y(:,Z_C==k), lambda, type);
end

%%% Load initial variables  %%%
Psi.C              = C;
Psi.Z_C            = Z_C;
Psi.lambda         = lambda;
Psi.alpha          = alpha;
Psi.type           = type;
Psi.table_members  = table_members;
Psi.table_logLiks  = table_logLiks;
Psi.MaxLogProb     = -inf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Run Gibbs Sampler for dd-CRP                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if options.verbose == 1
    fprintf('*** Initialized with %d clusters out of %d observations ***\n', K, N);
    fprintf('Running dd-CRP Mixture Sampler... \n');
    tic;
end
for i = 1:T
    
    
    %%% Draw Sample dd(SPCM)-CRP %%%
    tic;
    [Psi.C, Psi.Z_C, Psi.table_members, Psi.table_logLiks] = sample_ddCRPMM(Y, S_alpha, Psi); 
    
    %%% Compute the Posterior Conditional Probability of current Partition %%%    
    [LogProb data_LogLik] = logPr_spcmCRP(Y, S_alpha, Psi); 
    if options.verbose == 1
        fprintf('Iteration %d: Started with %d clusters ', i, max(Psi.Z_C));
        fprintf('--> moved to %d clusters with logprob = %4.2f\n', max(Psi.Z_C) , LogProb);
    end
    %%% Store Stats %%%
    Psi_Stats.CompTimes(i)     = toc;
    Psi_Stats.PostLogProbs(i)  = LogProb;
    Psi_Stats.LogLiks(i)       = data_LogLik;
    Psi_Stats.TableAssign(:,i) = Psi.Z_C;
    Psi_Stats.TotalClust(i)    = max(Psi.Z_C);
    
    %%% If current posterior is higher than previous update MAP estimate %%%
    if (Psi_Stats.PostLogProbs(i) > Psi.MaxLogProb)
        Psi.MaxLogProb = Psi_Stats.PostLogProbs(i);
        Psi.Maxiter = i;
    end    
end

%%% Sample table parameters with MAP estimate%%%
% Select MAP estimate for table assignments
Psi.Z_C = Psi_Stats.TableAssign(:,Psi.Maxiter);
% Eq. 5X in Appendix 
[Psi.Theta] = sample_TableParams(Y, Psi.Z_C, lambda, type);

if options.verbose == 1
    toc;
    fprintf('*************************************************************\n');
end
end
