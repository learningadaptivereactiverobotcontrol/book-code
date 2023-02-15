function [Priors Mu Sigma]=SEDS_Solver_Likelihood(Priors,Mu,Sigma,Data,options)
%
% SEDS optimization toolbox: version 1.7 issued on 13 May 2011
%
% This function finds a locally optimal value of a Gaussian Mixture Model
% under the constraint of ensuring its global asymptotic stability.
%
% This function should not be used directly. Please use SEDS_Solver
% function with the option options.objective = 'likelihood' instead.
%
% The function can be called using:
%    [Priors Mu Sigma]=SEDS_Solver_Likelihood(Priors_0,Mu_0,Sigma_0,Data,options)
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
d = size(Sigma,1)/2; %dimension of the model
K = size(Sigma,3); %number of Gaussian functions

%% Optimization

%transforming the GMM model into a vector of optimization parameters
p0 = GMM_2_Parameters(Priors,Mu,Sigma,d,K);

if strcmpi(options.criterion,'eigenvalue')
    ctr_handle = @(p) ctr_eigenvalue(p,d,K,options);
else
    ctr_handle = @(p) ctr_principal_minor(p,d,K,options);
end
options.ctr_handle = ctr_handle;
obj_handle = @(p)obj(p,Data,d,K,options);

% Running the optimization
if options.matlab_optimization %MATLAB optimization
    if options.display
        str = 'iter';
    else
        str = 'off';
    end
    
    % Options for NLP Solvers
    optNLP = optimset( 'Algorithm', 'interior-point', 'LargeScale', 'off',...
        'GradObj', 'on', 'GradConstr', 'on', 'DerivativeCheck', 'off', ...
        'Display', 'iter', 'TolX', 1e-12, 'TolFun', 1e-12, 'TolCon', 1e-12, ...
        'MaxFunEval', 200000, 'MaxIter', options.max_iter, 'DiffMinChange', ...
         options.tol_stopping, 'Hessian','off','display',str);

    % Solve fully-discretized optimal control problem
    if isinf(options.cons_penalty)
        popt = fmincon(obj_handle,p0,[],[],[],[],[],[],ctr_handle,optNLP);
    else
        popt = fminunc(obj_handle,p0,optNLP);
    end
else %Our SQP optimization
    popt = Optimize_SQP(obj_handle,ctr_handle,p0,Data,d,K,options);
end

% transforming back the optimization parameters into the GMM model
[Priors Mu Sigma] = Parameters_2_GMM(popt,d,K,options);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [J, dJ]=obj(p,Data,d,K,options)
% This function computes the derivative of the likelihood objective function
% w.r.t. optimization parameters.
nData = size(Data,2);
[Priors Mu Sigma A L] = shape_DS(p,d,K,options.tol_mat_bias);

for k=1:K
    tmp = Data' - repmat(Mu(:,k)',nData,1);
    prob = sum((tmp/Sigma(:,:,k)).*tmp, 2);
    Pxi(:,k) = exp(-0.5*prob) / sqrt((2*pi)^(2*d) * (abs(det(Sigma(:,:,k)))+realmin));
end
Pxi(Pxi==0) = realmin;

% computing dJ
dJ = zeros(size(p));
rSrs = zeros(2*d,2*d);
for k=1:K
    % Sensitivity of Obj w.r.t. Priors^k
    if options.perior_opt
        dJ(k) = -exp(-p(k))*Priors(k)^2*sum(Pxi(:,k)./(Pxi*Priors) -  1);
    end
    
    % Sensitivity of Obj w.r.t. Mu
    if options.mu_opt
        tmp = Sigma(:,:,k)\(Data - repmat(Mu(:,k),1,nData));
        dJ(K+(k-1)*d+1:K+k*d) = -[eye(d);A(:,:,k)]'*tmp*(Pxi(:,k)./(Pxi*Priors))*Priors(k);
    end

    % Sensitivity of Obj w.r.t. Sigma
    i_c=0;
    invSigma = eye(2*d)/Sigma(:,:,k);
    invSigma_x = eye(d)/Sigma(1:d,1:d,k);
    det_term = sign(det(Sigma(:,:,k)));
    tmp = Data' - repmat(Mu(:,k)',nData,1);
    for i=1:2*d
        for j=i:2*d
            i_c = i_c + 1;
            if options.sigma_x_opt || (i<=d && j>d)
                rSrs = rSrs *0;
                rSrs(j,i)=1;
                rSrs = rSrs*L(:,:,k)' + L(:,:,k)*rSrs';
            
                rArs = (-A(:,:,k) * rSrs(1:d,1:d) + rSrs(d+1:2*d,1:d)) ...
                       *invSigma_x * Mu(1:d,k);
                
                dJ(K+K*d+(k-1)*d*(2*d+1)+i_c) = dJ(K+K*d+(k-1)*d*(2*d+1)+i_c) ...
                    -( 0.5*sum(tmp*(invSigma*rSrs*invSigma).*tmp,2) + ...
                    -0.5*det_term*trace(invSigma*rSrs) + ... %derivative with respect to det Sigma which is in the numenator
                    (tmp*invSigma)*[zeros(d,1);rArs])' ... %since Mu_xi_d = A*Mu_xi, thus we should consider its effect here
                    *(Pxi(:,k)./(Pxi(:,:)*Priors))*Priors(k);
            end
        end
    end
end
if isinf(options.cons_penalty)
    J = -sum(log(Pxi*Priors))/nData;
    dJ = dJ/nData;
else
	%Computing the penalty for violating the constraints
    [c, tmp, dc]=options.ctr_handle(p);
    J = -sum(log(Pxi*Priors))/nData + options.cons_penalty*(c'*c);
    dJ = dJ/nData + 2*options.cons_penalty*(dc*c);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [c ceq dc dceq]=ctr_principal_minor(p,d,K,options)
% This function computes the derivative of the constrains w.r.t.
% optimization parameters.
ceq = [];
dceq = [];
c  = zeros(K*d,1);
dc = zeros(length(p),K*d);
[tmp tmp Sigma A L] = shape_DS(p,d,K,options.tol_mat_bias);

rSrs = zeros(2*d,2*d);
for k=1:K
    invSigma_x = eye(d)/Sigma(1:d,1:d,k);
    B = A(:,:,k)+A(:,:,k)';
    for j = 1:d
        %conditions on negative definitenes of A
        if (-1)^(j+1)*det(B(1:j,1:j))+(options.tol_mat_bias)^(j/d) > 0  || isinf(options.cons_penalty)
            c((k-1)*d+j)=(-1)^(j+1)*det(B(1:j,1:j))+(options.tol_mat_bias)^(j/d);
        end
        
        %computing the sensitivity of the constraints to the parameters
        if nargout > 2
            i_c = 0;
            for i1=1:d
                for i2=i1:2*d
                    i_c = i_c +1;
                    if options.sigma_x_opt || i1>d || i2>d
                        rSrs = rSrs * 0;
                        rSrs (i2,i1) = 1;

                        rSrs = rSrs*L(:,:,k)' + L(:,:,k)*rSrs';
                        rArs = (-A(:,:,k) * rSrs(1:d,1:d) + rSrs(d+1:2*d,1:d)) * invSigma_x;
                        rBrs = rArs + rArs';
                        if j==1
                            dc((d+1)*K + (k-1)*d*(2*d+1)+i_c,(k-1)*d+j) = rBrs(1,1);
                        else
                            tmp = det(B(1:j,1:j));
                            if abs(tmp) > 1e-10
                                term = trace(B(1:j,1:j)\rBrs(1:j,1:j))*tmp;
                            else
                                term = trace(adjugate(B(1:j,1:j))*rBrs(1:j,1:j));
                            end
                            dc((d+1)*K + (k-1)*d*(2*d+1)+i_c,(k-1)*d+j) = (-1)^(j+1)*term;
                        end
                    end
                end
            end
        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [c ceq dc dceq]=ctr_eigenvalue(p,d,K,options)
% This function computes the derivative of the constrains w.r.t.
% optimization parameters.

[tmp tmp Sigma A L] = shape_DS(p,d,K,options.tol_mat_bias);
ceq = [];
dceq = [];
c  = zeros(K*d,1);
dc = zeros(length(p),K*d);
delta = 10^-5;
rSrs = zeros(2*d,2*d);

for k=1:K
    B = A(:,:,k)+A(:,:,k)';
    [V lambda] = eig(B);
    [lambda ind] = sort(diag(lambda));
    c((k-1)*d+1:k*d)=lambda;
    
    if nargout > 2
        i_c = K + d*K + (k-1)*d*(2*d+1);
        
        if all(sum((repmat(lambda,1,d) - repmat(lambda,1,d)')==0)==1) %the eigenvalue is not repeated
            V = V(:,ind);
            invSigma_x = eye(d)/Sigma(1:d,1:d,k);
            ch_direct = true;
        else
            ch_direct = false;
        end
        
        for i=1:2*d
            for j=i:2*d
                i_c = i_c + 1;
                if options.sigma_x_opt || (i<=d && j>d)
                    if ch_direct %the eigenvalue is not repeated
                        rSrs = rSrs * 0;
                        rSrs (j,i) = 1;

                        rSrs = rSrs*L(:,:,k)' + L(:,:,k)*rSrs';
                        rArs = (-A(:,:,k) * rSrs(1:d,1:d) + rSrs(d+1:2*d,1:d)) * invSigma_x;
                        rBrs = rArs + rArs';
                        for ii=1:d
                            dc(i_c,(k-1)*d+ii) = V(:,ii)'*rBrs*V(:,ii);
                        end                        
                    else %computing using finite difference
                        Ltmp = L(:,:,k);
                        Ltmp(j,i) = Ltmp(j,i) + delta;
                        Sigma_k = Ltmp*Ltmp';
                        Btmp = Sigma_k(d+1:2*d,1:d)/Sigma_k(1:d,1:d);
                        Btmp = Btmp + Btmp';
                        lambda2 = eig(Btmp);
                        lambda_d = (lambda2-lambda)/delta;
                        dc(i_c,(k-1)*d+1:k*d) = lambda_d;
                    end
                end
            end
        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p0 = GMM_2_Parameters(Priors,Mu,Sigma,d,K)
% transforming optimization parameters into a column vector
p0 = [-log(1./Priors(:)-1);reshape(Mu(1:d,:),[],1)];
% p0 = [log(Priors(:));reshape(Mu(1:d,:),[],1)];
for k=1:K
    Sigma(:,:,k) = chol(Sigma(:,:,k))';
    for i=1:2*d
        p0 = [p0;Sigma(i:2*d,i,k)];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors Mu Sigma] = Parameters_2_GMM(popt,d,K,options)
% transforming the column of parameters into Priors, Mu, and Sigma
[Priors Mu Sigma] = shape_DS(popt,d,K,options.tol_mat_bias);
if options.normalization
    for k=1:K
        Sigma(:,:,k) = options.Wn\Sigma(:,:,k)/options.Wn;
        Mu(1:d,k) = options.Wn(1:d,1:d)\Mu(1:d,k);
        Mu(d+1:2*d,k) = Sigma(d+1:2*d,1:d,k)/Sigma(1:d,1:d,k)*Mu(1:d,k);
        Sigma(1:d,d+1:2*d,k) = Sigma(d+1:2*d,1:d,k)';
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors Mu Sigma A L] = shape_DS(p,d,K,Tol)
if K==1
    Priors = 1;
else
    Priors = 1./(1+exp(-p(1:K)));
    Priors = Priors/sum(Priors);
end

Mu_x = reshape(p(K+1:K+d*K),d,K);

% this function form Sigma from a column vector of parameters p
L = zeros(2*d,2*d,K);
Sigma = zeros(2*d,2*d,K);
i_c = K+d*K+1;
for k=1:K
    for i=1:2*d
        L(i:2*d,i,k) = p(i_c:i_c+2*d-i);
        i_c = i_c + 2*d - i + 1;
    end
    Sigma(:,:,k) = L(:,:,k)*L(:,:,k)' + Tol;
    
    A(:,:,k) = Sigma(d+1:end,1:d,k)/Sigma(1:d,1:d,k);
    
    % Based on the second stability conditions
    Mu_xd(:,k) = A(:,:,k)*Mu_x(:,k);
end
Mu=[Mu_x;Mu_xd];