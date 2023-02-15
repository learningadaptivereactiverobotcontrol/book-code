function [Priors Mu Sigma]=SEDS_Solver(Priors,Mu,Sigma,Data,varargin)
%
% SEDS optimization toolbox: version 1.95 issued on 12 Feb. 2013
%
% This function finds a locally optimal value of a Gaussian Mixture Model
% under the constraint of ensuring its global asymptotic stability.
%
% The function can be called using:
%       [Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data)
%
% or
%       [Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,options)
%
% to also pass a structure of desired options.
%
% Important NOTE: Both the demonstration data, and the model estimation
% should be in the target frame of reference. In other words, this codes
% assumes that the target is at the origin!
%
% Inputs -----------------------------------------------------------------
%
%   o Priors_0:  1 x K array representing an initial guess for prior
%                probabilities of the K GMM components.
%
%   o Mu_0:      2d x K array representing an initial guess for centers of
%                the K GMM components.
%
%   o Sigma_0:   2d x 2d x K array representing an initial guess for
%                covariance matrices of the K GMM components.
%
%   o Data:      A 2d x N_Total matrix containing all demonstration data points.
%                Rows 1:d corresponds to trajectories and the rows d+1:2d
%                are their first time derivatives. Each column of Data stands
%                for a datapoint. All demonstrations are put next to each other 
%                along the second dimension. For example, if we have 3 demos
%                D1, D2, and D3, then the matrix Data is:
%                                 Data = [[D1] [D2] [D3]]
%
%   o options: A structure to set the optional parameters of the solver.
%              The following parameters can be set in the options:
%       - .tol_mat_bias:     a very small positive scalar to avoid
%                            instabilities in Gaussian kernel [default: 10^-15]
%
%       - .tol_stopping:     A small positive scalar defining the stoppping
%                            tolerance for the optimization solver [default: 10^-10]
%
%       - .i_max:            maximum number of iteration for the solver [default: i_max=1000]
%
%       - .objective:        'likelihood': use likelihood as criterion to
%                            optimize parameters of GMM
%                            'mse': use mean square error as criterion to
%                            optimize parameters of GMM
%                            'direction': minimize the angle between the
%                            estimations and demonstrations (the velocity part)
%                            to optimize parameters of GMM
%                            [default: 'mse']
%
%       - .criterion:        The criterion that should be used to verify the 
%                            negative definiteness of matrices. There are
%                            two options: 'eigenvalue' or 'principal_minor'.
%                            [default: 'mse']
%
%       - .display:          An option to control whether the algorithm
%                            displays the output of each iterations [default: true]
%
%       - .matlab_optimization: If true, SEDS_Solver uses MATLAB
%                            optimization toolbox. Otherwise it uses the
%                            pre-built optimization in SEDS package [default: true]
%                            Each toolbox has its own advantages, and not
%                            necessary converges to the same result. You may
%                            try the one that suits more for you.
%
%       - .perior_opt:       Shall the solver optimize priors? This is an
%                            option given to the user if s/he wishes not to
%                            optimize the priors [default: true]
%
%       - .mu_opt:           Shall the solver optimize centers? This is an
%                            option given to the user if s/he wishes not to
%                            optimize the centers Mu [default: true]
%
%       - .sigma_x_opt:      Shall the solver optimize Sigma_x? This is an
%                            option given to the user if s/he wishes not to
%                            optimize the Sigma_x [default: true]
%
%       - .normalization:    Activating the normalization options usually 
%                            improves the learning performance, especially
%                            for the case where the variance of the data is
%                            big along some dimensions. This option is not
%                            completely tested! [default: false]
%
%       - .cons_penalty:     Most of the time, it is preferable to
%                            transform a constrained optimization problem
%                            into an unconstrained one by penalizing if the
%                            constrains are violated. 'cons_penalty' should
%                            be a big value in comparison to the order of
%                            magnitude of data. If you wish to solve the
%                            real unconstrained problem, set the value of
%                            'cons_penalty' to Inf [default: 10^4]
%
% Outputs ----------------------------------------------------------------
%
%   o Priors:  1 x K array representing the prior probabilities of the K GMM 
%              components.
%
%   o Mu:      2d x K array representing the centers of the K GMM components.
%
%   o Sigma:   2d x 2d x K array representing the covariance matrices of the 
%              K GMM components.
%
%  NOTE: The final model is in the target frame of reference.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2010 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The program is free for non-commercial academic use. Please contact the
% author if you are interested in using the software for commercial purposes.
% The software must not be modified or distributed without prior permission
% of the authors. Please acknowledge the authors in any academic publications
% that have made use of this code or part of it. Please use this BibTex
% reference:
% 
%   S. M. Khansari Zadeh and A. Billard, "Imitation learning of Globally 
%   Stable Non-Linear Point-to-Point Robot Motions using Nonlinear
%   Programming", in Proceeding of the 2010 IEEE/RSJ International
%   Conference on Intelligent Robots and Systems (IROS 2010), Taipei,
%   Taiwan, October 2010 
%
% To get latest upadate of the software please visit
%                          http://lasa.epfl.ch/khansari
%
% Please send your feedbacks or questions to:
%                           mohammad.khansari_at_epfl.ch

%% initializing ...

% parsing options
if isempty(varargin)
    options = check_options();
else
    options = check_options(varargin{1}); % Checking the given options, and add ones are not defined.
end

d = size(Sigma,1)/2; %dimension of the model
K = size(Sigma,3); %number of Gaussian functions

if size(Mu,1)~=2*d || size(Data,1)~=2*d || size(Mu,2)~=K || size(Sigma,3)~=K || length(Priors)~=K
    disp('Exiting the solver with an error.')
    disp('Error in dimensionality of the input variables!')
    disp('Note the following relation should be held:')
    disp('             size(Mu,1)=size(Sigma,1)=size(Data,1)')
    disp('             size(Mu,2)=size(Sigma,3)=length(Priors)')
    return
end

%% Optimization

if K==1 %there is no need to optimize Priors
    options.perior_opt = 0;
end

if options.normalization %normalizing the model based on the variance of the data
    [Mu Sigma Data options] = Normalize_Model(Mu,Sigma,Data,options);
end

if strcmpi(options.objective,'mse')
    [Priors Mu Sigma]=SEDS_Solver_MSE(Priors,Mu,Sigma,Data,options);
elseif strcmpi(options.objective,'direction')
    [Priors Mu Sigma]=SEDS_Solver_Direction(Priors,Mu,Sigma,Data,options);
else
    [Priors Mu Sigma]=SEDS_Solver_Likelihood(Priors,Mu,Sigma,Data,options);
end

% Just to check if every thing goes well :)
if options.display
    check_constraints(Sigma);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Mu Sigma Data options] = Normalize_Model(Mu,Sigma,Data,options)
d = size(Mu,1)/2;
K = size(Mu,2);
W_x  = 100/sqrt(var(sqrt(sum(Data(1:d,:).*Data(1:d,:),1))));
W_xd = 100/sqrt(var(sqrt(sum(Data(d+1:2*d,:).*Data(d+1:2*d,:)))));
options.Wn = diag([ones(1,d)*W_x ones(1,d)*W_xd]); %Normalization matrix. Usually normalization improves the optimization performance.

Data = options.Wn*Data;
% transforming optimization parameters into a column vector
Mu = options.Wn*Mu;
for k=1:K
    Sigma(:,:,k) = options.Wn*Sigma(:,:,k)*options.Wn;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function options = check_options(varargin)
% parsing options
if ~isempty(varargin)
    options = varargin{1};
end
    
if isempty(varargin) || ~isfield(options,'tol_mat_bias')
    options.tol_mat_bias = 10^(-15);
end
if ~isfield(options,'cons_penalty')
    options.cons_penalty = inf;
end
if ~isfield(options,'perior_opt')
    options.perior_opt = 1;
else 
    options.perior_opt = options.perior_opt > 0;
end
if ~isfield(options,'mu_opt')
    options.mu_opt = 1;
else 
    options.mu_opt = options.mu_opt > 0;
end
if ~isfield(options,'sigma_x_opt')
    options.sigma_x_opt = 1;
else 
    options.sigma_x_opt = options.sigma_x_opt > 0;
end
if ~isfield(options,'tol_stopping')
    options.tol_stopping = 10^-10;
end
if ~isfield(options,'max_iter')
    options.max_iter = 1000;
end
if ~isfield(options,'matlab_optimization')
    options.matlab_optimization = 1;
else 
    options.matlab_optimization = options.matlab_optimization > 0;
end
if ~isfield(options,'display')
    options.display = 1;
else 
    options.display = options.display > 0;
end
if ~isfield(options,'normalization')
    options.normalization = 0;
else 
    options.normalization = options.normalization > 0;
end
if ~isfield(options,'Wn')
    options.Wn = 1;
end
if ~isfield(options,'objective')
    options.objective='mse';
else
    if ~strcmpi(options.objective,'mse') && ~strcmpi(options.objective,'likelihood') && ~strcmpi(options.objective,'direction')
        disp('Unknown objective function. Considering MSE as the objective function')
        disp('Press enter to continue.')
        options.objective='mse';
        pause;
    end
end
if ~isfield(options,'criterion')
    options.criterion='eigenvalue';
else
    if ~strcmpi(options.criterion,'eigenvalue') && ~strcmpi(options.criterion,'principal_minor')
        disp('Unknown constraint criterion. Considering eigenvalue as the objective function')
        disp('Press enter to continue.')
        options.criterion='eigenvalue';
        pause;
    end
end

% if strcmpi(options.criterion,'eigenvalue') && ~options.matlab_optimization
%     disp('Eigenvalue criteria can only be used with matlab optimization');
%     disp('Modifying the user option -> options.matlab_optimization = 1');
%     options.matlab_optimization = 1;
%     disp('Press enter to continue.')
%     pause;
% end
% if strcmpi(options.criterion,'eigenvalue') && ~isinf(options.cons_penalty)
%     disp('Eigenvalue criteria cannot be used with the penalty function');
%     disp('Modifying the user option -> options.cons_penalty=inf');
%     options.cons_penalty=inf;
%     disp('Press enter to continue.')
%     pause;
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function k = check_constraints(Sigma)
% checking if every thing goes well. Sometimes if the parameter
% 'options.cons_penalty' is not big enough, the constrains may be violated.
% Then this function notifies the user to increase 'options.cons_penalty'.
ch = true;
d = size(Sigma,1)/2;
err = [];
i = 1;
for k=1:size(Sigma,3)
    A = Sigma(d+1:end,1:d,k)/Sigma(1:d,1:d,k);
    if any(sign(eig(A+A')) >= 0 )
    	err(i,:) = [k;eig((A+A')/2)];
        i = i+1;
    end
end
if isempty(err)
    disp('Optimization finished successfully.')
    disp(' ')
    disp(' ')
else
    disp(' ')
    disp(' ')
    disp('Optimization did not reach to an optimal point.')
    disp('Some constraints were slightly violated.')
    disp('The error may be due to change of hard constraints to soft constrints.')
    disp('To handle this error, increase the value of ''cons_penalty'' and re-run the optimization.')
    disp('For debugging purpose:')
    disp('  error :')
    disp(err)
    disp(' ')
end    