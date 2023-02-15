function [Priors Mu Sigma]=SEDS_Solver_MSE(Priors_0,Mu_0,Sigma_0,Data,options)
%
% SEDS optimization toolbox: version 1.7 issued on 13 May 2011
%
% This function finds a locally optimal value of a Gaussian Mixture Model
% under the constraint of ensuring its global asymptotic stability.
%
% This function should not be used directly. Please use SEDS_Solver
% function with the option options.objective = 'mse' instead.
%
% The function can be called using:
%       [Priors Mu Sigma]=SEDS_Solver_MSE(Priors_0,Mu_0,Sigma_0,Data,options)
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

d = size(Sigma_0,1)/2; %dimension of the model
K = size(Sigma_0,3); %number of Gaussian functions

%% Optimization

%transforming the GMM model into a vector of optimization parameters
p0 = GMM_2_Parameters(Priors_0,Mu_0,Sigma_0,d,K);

if strcmpi(options.criterion,'eigenvalue')
    ctr_handle = @(p) ctr_eigenvalue(p,d,K,options.tol_mat_bias);
else
    ctr_handle = @(p) ctr_principal_minor(p,d,K,options);
end
options.ctr_handle = ctr_handle;
obj_handle = @(p)obj(p,Data,d,K,options);

% Running the optimization
if options.matlab_optimization
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
        popt = fmincon(obj_handle, p0,[],[],[],[],[],[],ctr_handle,optNLP);
    else
        popt = fminunc(obj_handle, p0,optNLP);
    end
else
    popt = Optimize_SQP(obj_handle,ctr_handle,p0,Data,d,K,options);
end

% transforming back the optimization parameters into the GMM model
[Priors Mu Sigma] = Parameters_2_GMM(popt,d,K,options);
Priors = Priors/sum(Priors);
% Sigma(d+1:2*d,d+1:2*d,:) = Sigma_0(d+1:2*d,d+1:2*d,:);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [J, dJ]=obj(p,Data,d,K,options)
% This function computes the derivative of the likelihood objective function
% w.r.t. optimization parameters.
nData = size(Data,2);
[Priors Mu Sigma A L_x] = shape_DS(p,d,K,options.tol_mat_bias);

x = Data(1:d,:);
[xd, tmp, h]= GMR_SEDS(Priors,Mu,Sigma,x,1:d,d+1:2*d);
% h(h==0) = realmin;

dJdxd = xd-Data(d+1:2*d,:); %derivative of J w.r.t. xd
dJ = zeros(size(p));
if nargout > 1 
    % computing dJ
    rSrs = zeros(d,d);
    for k=1:K
        %since we use these eq. a lot. It is better to compute them once
        sum_tmp = sum((A(:,:,k)*x-xd).*dJdxd); 
        tmp_x = x - repmat(Mu(1:d,k),1,nData);
        invSigma_x = eye(d)/Sigma(1:d,1:d,k);

        % Sensitivity of Obj w.r.t. Priors^k
        if options.perior_opt
            dJ(k) = exp(-p(k))*Priors(k)*sum(h(:,k)'.*sum_tmp); %derivative of xd w.r.t. priors(k)
        end


        % Sensitivity of Obj w.r.t. Mu
        if options.mu_opt
    %         dJ(K+(k-1)*d+1:K+k*d) = (h(:,k)'.*sum_tmp)*((x-repmat(Mu(1:d,k),1,nData))'/Sigma(1:d,1:d,k));
            dJ(K+(k-1)*d+1:K+k*d) = invSigma_x*tmp_x*(h(:,k).*sum_tmp');
        end

        % Sensitivity of Obj w.r.t. Sigma
        i_c=0;
        i_a = d*(d+1)/2;
        for i=1:d
            for j=1:d
                if options.sigma_x_opt && j>=i %finding dJ w.r.t. Sigma_x parameters
                    i_c = i_c + 1;
                    rSrs = rSrs *0;
                    rSrs(j,i)=1;
                    rSrs = rSrs*L_x(:,:,k)' + L_x(:,:,k)*rSrs';

                    dJ(K+K*d+(k-1)*d*(3*d+1)/2+i_c) = 0.5*sum(...
                          (sum((invSigma_x*rSrs*invSigma_x*tmp_x).*tmp_x) + ... %derivative w.r.t. Sigma in exponential
                              -trace(invSigma_x*rSrs)).*sum_tmp.*h(:,k)'); %derivative with respect to det Sigma which is in the numenator

                end

                %finding dJ w.r.t. Sigma_xdx parameters
                rSrs = rSrs *0;
                rSrs(j,i)=1;
                i_a = i_a + 1;
                dJ(K+K*d+(k-1)*d*(3*d+1)/2+i_a) = sum(sum((rSrs*x).*dJdxd).*h(:,k)');  %derivative of A
            end
        end

    end
end
if isinf(options.cons_penalty)
    J = 0.5*sum(sum(dJdxd.*dJdxd))/nData;
    dJ = dJ/nData;
else
    %Computing the penalty for violating the constraints
    [c, tmp, dc]=options.ctr_handle(p);
    J = 0.5*sum(sum(dJdxd.*dJdxd))/nData + options.cons_penalty*(c'*c);
    dJ = dJ/nData + 2*options.cons_penalty*(dc*c);
end


function [c ceq dc dceq]=ctr_principal_minor(p,d,K,options)
% This function computes the derivative of the constrains w.r.t.
% optimization parameters.
[tmp tmp tmp A] = shape_DS(p,d,K,options.tol_mat_bias);
ceq = [];
dceq = [];
c  = zeros(K*d,1);
dc = zeros(length(p),K*d);
rArs = zeros(d,d);

for k=1:K
    B = A(:,:,k)+A(:,:,k)';
    for j = 1:d %number of constraints for each DS
        %conditions on negative definitenes of A
        if (-1)^(j+1)*det(B(1:j,1:j))+(options.tol_mat_bias)^(j/d) > 0  || isinf(options.cons_penalty)
            c((k-1)*d+j)=(-1)^(j+1)*det(B(1:j,1:j))+(options.tol_mat_bias)^(j/d);
        end
        
        if nargout > 2
            %computing the sensitivity of the constraints to the parameters
            i_c = d*(d+1)/2; %we skip derivative of A w.r.t. parameters of Sigma_x
            for i1=1:j
                for i2=1:j
                    i_c = i_c +1;
                    rArs = rArs * 0;
                    rArs (i2,i1) = 1;
                    rBrs = rArs + rArs';
                    if j==1
                        dc((d+1)*K + (k-1)*d*(3*d+1)/2+i_c,(k-1)*d+j) = rBrs(1,1);
                    else
                        tmp = det(B(1:j,1:j));
                        if abs(tmp) > 1e-10
                            term = trace(B(1:j,1:j)\rBrs(1:j,1:j))*tmp;
                        else
                            term = trace(adjugate(B(1:j,1:j))*rBrs(1:j,1:j));
                        end
                        dc((d+1)*K + (k-1)*d*(3*d+1)/2+i_c,(k-1)*d+j) = (-1)^(j+1)*term;
                    end
                end
            end
        end
    end
end

function [c ceq dc dceq]=ctr_eigenvalue(p,d,K,tol_mat_bias)
% This function computes the derivative of the constrains w.r.t.
% optimization parameters.
[tmp tmp tmp A] = shape_DS(p,d,K,tol_mat_bias);
ceq = [];
dceq = [];
c  = zeros(K*d,1);
dc = zeros(length(p),K*d);
delta = 10^-5;

for k=1:K
    B = A(:,:,k)+A(:,:,k)';
    [V lambda] = eig(B);
    [lambda ind] = sort(diag(lambda));
    c((k-1)*d+1:k*d)=lambda;
    
    if nargout > 2
        i_c = K + d*K + (k-1)*(d/2*(d+1)+d*d) + d/2*(d+1);
        
        if all(sum((repmat(lambda,1,d) - repmat(lambda,1,d)')==0)==1) %the eigenvalue is not repeated
            V = V(:,ind);
            for i=1:d
                dLambda_i = V(:,i)*V(:,i)';
                dc(i_c+1:i_c+d*d,(k-1)*d+i) = dLambda_i(:)*2;
            end
        else %computing using finite difference
            for i=1:d
                for j=1:d
                    Btmp = B;
                    if i==j
                        Btmp(i,i) = Btmp(i,i) + 2*delta;
                    else
                        Btmp(j,i) = Btmp(j,i) + delta;
                        Btmp(i,j) = Btmp(i,j) + delta;
                    end
                    lambda2 = eig(Btmp);
                    lambda_d = (lambda2-lambda)/delta;
                    i_c = i_c + 1;
                    dc(i_c,(k-1)*d+1:k*d) = lambda_d;
                end
            end
        end
    end
    
%     if nargout > 1
%         i_c = K + d*K + (k-1)*(d/2*(d+1)+d*d) + d/2*(d+1);
%         for i=1:d
%             for j=1:d
%                 Btmp = B;
%                 if i==j
%                     Btmp(i,i) = Btmp(i,i) + 2*delta;
%                 else
%                     Btmp(j,i) = Btmp(j,i) + delta;
%                     Btmp(i,j) = Btmp(i,j) + delta;
%                 end
%                 lambda2 = real(eig(Btmp));
%                 lambda_d = (lambda2-lambda)/delta;
%                 i_c = i_c + 1;
%                 dc(i_c,(k-1)*d+1:k*d) = lambda_d;
%             end
%         end
%     end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors Mu Sigma A L_x] = shape_DS(p,d,K,tol_mat_bias)
% transforming the column of parameters into Priors, Mu, and Sigma
L_x = zeros(d,d,K);
Mu = zeros(2*d,K);
Sigma = zeros(2*d,2*d,K);
A = zeros(d,d,K);

if K==1
    Priors = 1;
else
    Priors = 1./(1+exp(-p(1:K)));
end
% Priors = p(1:K);
% Priors = exp(Priors)/sum(exp(Priors)); %to ensure that it is always positive
% p(p(1:K)>200) = 200; %to avoid numerical issue
% p(p(1:K)<-200) = -200; %to avoid numerical issue
% Priors = p(1:K);
% Priors = exp(Priors)/sum(exp(Priors)); %to ensure that it is always positive

Mu(1:d,:) = reshape(p(K+1:K+d*K),d,K);

i_c = K+d*K+1;
for k=1:K
    for i=1:d
        L_x(i:d,i,k) = p(i_c:i_c+d-i);
        i_c = i_c + d - i + 1;
    end
    A(:,:,k) = reshape(p(i_c:i_c+d^2-1),d,d);
    i_c = i_c + d^2;
    Sigma(1:d,1:d,k) = L_x(:,:,k)*L_x(:,:,k)' + tol_mat_bias*eye(d);
    Sigma(d+1:2*d,1:d,k) = A(:,:,k)*Sigma(1:d,1:d,k);
    Mu(d+1:2*d,k) = A(:,:,k)*Mu(1:d,k);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p0 = GMM_2_Parameters(Priors,Mu,Sigma,d,K)
% transforming optimization parameters into a column vector
p0 = [-log(1./Priors(:)-1);reshape(Mu(1:d,:),[],1)];
% p0 = [log(Priors(:));reshape(Mu(1:d,:),[],1)];
for k=1:K
    tmp_mat = chol(Sigma(1:d,1:d,k))';
    for i=1:d
        p0 = [p0;tmp_mat(i:d,i)]; %#ok<*AGROW>
    end
    A_tmp = Sigma(d+1:2*d,1:d,k)/Sigma(1:d,1:d,k);
    p0 = [p0;reshape(A_tmp,[],1)];
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