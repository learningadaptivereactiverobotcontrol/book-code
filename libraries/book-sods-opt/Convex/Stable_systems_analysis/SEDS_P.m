function [Priors, Mu, Sigma,A,Time]=SEDS_P(Data,K)


addpath([pwd, '/Stable_systems_analysis/SEDS_lib']);
addpath([pwd, '/Stable_systems_analysis/GMR_lib']);
% Pre-processing
dt = 0.1; %The time step of the demonstrations
tol_cutting = 1; % A threshold on velocity that will be used for trimming demos

% Training parameters
options.tol_mat_bias = 10^-6;
options.display = 1;  
options.tol_stopping=10^-10;
options.max_iter = 5000;

% options.objective = 'likelihood';
options.objective = 'mse';
options.display='off';

%% SEDS learning algorithm
[Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,K); %finding an initial guess for GMM's parameter
tic
[Priors, Mu, Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,options); %running SEDS optimization solver
Time=toc;
d=size(Sigma,1)/2;
for o=1:K
    Sigmainput=Sigma(1:d,1:d,o);
    SigmaInput_Output=Sigma((d+1):2*d,1:d,o);
    A(:,:,o)=SigmaInput_Output*inv(Sigmainput);
end

MEAN_SQ_ER(Data,Priors,Mu,Sigma,A);
