function [Priors Mu Sigma]=SEDS_Solver_Direction(Priors_0,Mu_0,Sigma_0,Data,options)
%
% SEDS optimization toolbox: version 1.95 issued on 12 Feb. 2013
%
% This function finds a locally optimal value of a Gaussian Mixture Model
% under the constraint of ensuring its global asymptotic stability.
%
% This function should not be used directly. Please use SEDS_Solver
% function with the option options.objective = 'direction' instead.
%
% The function can be called using:
%       [Priors Mu Sigma]=SEDS_Solver_Direction(Priors_0,Mu_0,Sigma_0,Data,options)
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
in = 1:d;
out = d+1:2*d;
K = size(Sigma_0,3); %number of Gaussian functions

C = eye(d);

%% Optimization

%transforming the GMM model into a vector of optimization parameters
p0 = GMM_2_Parameters(Priors_0,Mu_0,Sigma_0,K,in,out);

%Setting a defailt value, if the user does not provide the weighting coef. for the data points
if ~isfield(options,'weight')
    options.weight = 1;
end


if strcmpi(options.criterion,'eigenvalue')
    ctr_handle = @(p) ctr_eigenvalue(p,C,K,in,out);
else
    ctr_handle = @(p) ctr_principal_minor(p,C,d,K,in,out,options);
end
options.ctr_handle = ctr_handle;
obj_handle = @(p)obj(p,Data,K,in,out,options);

% Running the optimization
if options.matlab_optimization
    if options.display
        str = 'iter';
    else
        str = 'off';
    end
    
    % Options for NLP Solvers
    optNLP = optimset( 'Algorithm', 'interior-point', 'LargeScale', 'off',...
        'GradObj', 'off', 'GradConstr', 'on', 'DerivativeCheck', 'off', ...
        'Display', 'iter', 'TolX', options.tol_stopping, 'TolFun', options.tol_stopping, 'TolCon', 1e-12, ...
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
[Priors Mu Sigma] = Parameters_2_GMM(popt,K,in,out,options);
Priors = Priors/sum(Priors);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [J, dJ]=obj(p,Data,K,in,out,options)
% This function computes the derivative of the likelihood objective function
% w.r.t. optimization parameters.
n_in = length(in);
n_out = length(out);
nData = size(Data,2);
[Priors Mu Sigma A] = shape_DS(p,K,in,out);

x = Data(in,:);
[xd, h]= GMR_SEDS2(Priors,Mu,Sigma,x,in,out);
xd_data = Data(out,:);

J = sum(xd.*xd_data,1);
den = Mat_Vec_Norm(xd).*Mat_Vec_Norm(xd_data);
J = ((options.weight).*J)./den;
J(den == 0) = 0;
J = -sum(J);

if isinf(options.cons_penalty)
    if options.normalization
        J = J/nData/det(options.Wn(out,out));
    else
        J = J/nData;
    end
else
    %Computing the penalty for violating the constraints
    [c, tmp]=options.ctr_handle(p);
    J = J/nData + options.cons_penalty*(c'*c);
end

dJ = [];


function [c ceq dc dceq]=ctr_principal_minor(p,C,d,K,in,out,options)
% This function computes the derivative of the constrains w.r.t.
% optimization parameters.
[tmp tmp Sigma A] = shape_DS(p,K,in,out);
ceq = [];
dceq = [];
c  = zeros(2*K*d,1);
dc = zeros(length(p),2*K*d);
rArs = zeros(d,d);

for k=1:K
    B = A(:,:,k)'*C + C*A(:,:,k);
    for j = 1:d %number of constraints for each DS
        %conditions on negative definitenes of A
        c((k-1)*d+j)=(-1)^(j+1)*det(B(1:j,1:j))+(options.tol_mat_bias)^(j/d);
        
        if nargout > 2
            %computing the sensitivity of the constraints to the parameters
            i_c = (d+1)*K + (k-1)*d*(3*d+1)/2+d*(d+1)/2; %we skip derivative of A w.r.t. parameters of Sigma_x
            for i1=1:d
                for i2=1:d
                    i_c = i_c + 1;
                    rArs = rArs * 0;
                    rArs (i2,i1) = 1;
                    rBrs = C*rArs + rArs'*C;
                    if j==1
                        dc(i_c,(k-1)*d+j) = rBrs(1,1);
                    else
                        tmp = det(B(1:j,1:j));
                        if abs(tmp) > 1e-10
                            term = trace(B(1:j,1:j)\rBrs(1:j,1:j))*tmp;
                        else
                            term = trace(adjugate(B(1:j,1:j))*rBrs(1:j,1:j));
                        end
                        dc(i_c,(k-1)*d+j) = (-1)^(j+1)*term;
                    end
                end
            end
        end
    end
    
    
    
    for j = 1:d %number of constraints for each DS
        %conditions on positive definitenes of Sigma
        c(K*d+(k-1)*d+j)= - det(Sigma(1:j,1:j,k));
        
        if nargout > 2
            %computing the sensitivity of the constraints to the parameters
            i_c = (d+1)*K + (k-1)*d*(3*d+1)/2;
            for i1=1:j
                for i2=i1:j
                    i_c = i_c + 1;
                    rArs = rArs * 0;
                    rArs (i2,i1) = 1;
                    rBrs = rArs + rArs';
                    if j==1
                        dc(i_c,K*d+(k-1)*d+j) = -rBrs(1,1);
                    else
                        tmp = det(Sigma(1:j,1:j,k));
                        if abs(tmp) > 1e-10
                            term = trace(Sigma(1:j,1:j,k)\rBrs(1:j,1:j))*tmp;
                        else
                            term = trace(adjugate(Sigma(1:j,1:j,k))*rBrs(1:j,1:j));
                        end
                        dc(i_c,K*d+(k-1)*d+j) = -term;
                    end
                end
            end
        end
    end
   
end



function [c ceq dc dceq]=ctr_eigenvalue(p,C,K,in,out)
% This function computes the derivative of the constrains w.r.t.
% optimization parameters.
[tmp tmp Sigma A] = shape_DS(p,K,in,out);
delta = 10^-6;
n_in = length(in);
n_out = length(out);
d = n_out;
rAra = zeros(d,d);

c  = zeros(2*K*d,1);
dc = zeros(length(p),2*K*d);
ceq = [];
dceq = [];

for k=1:K
    B = A(:,1:d,k)'*C + C*A(:,1:d,k);
    
    [V lambda] = eig(B);
    [lambda ind] = sort(diag(lambda));
    c((k-1)*d+1:k*d) = lambda ;%options.tol_mat_bias^(-d);
    
    if nargout > 2
        i_a = K+K*n_in+k*(n_in*(n_in+1)/2) + (k-1)*(n_out*n_in);
        
        if all(sum(abs(repmat(lambda,1,d) - repmat(lambda,1,d)') < 1e-5)==1) %the eigenvalue is not repeated
            V = V(:,ind);
            for i=1:d
                for i1=1:d
                    for i2=1:d
                        rAra = rAra*0;
                        rAra(i2,i1)=1;
                        rAra = rAra'*C + C*rAra;
                        dLambda_i = V(:,i)'*rAra*V(:,i);
                        dc(i_a+(i1-1)*d+i2,(k-1)*d+i) = dLambda_i;
                    end
                end
            end
        else %computing using finite difference
            for i=1:d
                for j=1:d
                    Btmp = A(:,1:d,k);
                    Btmp(j,i) = Btmp(j,i) + delta;
                    Btmp = Btmp'*C + C*Btmp;
                    lambda2 = sort(eig(Btmp));
                    lambda_d = (lambda2-lambda)/delta;
                    i_a = i_a + 1;
                    dc(i_a,(k-1)*d+1:k*d) = lambda_d;
                end
            end
        end
    end
    
    [V lambda] = eig(Sigma(in,in,k));
    [lambda ind] = sort(diag(lambda));
    c(K*d+(k-1)*n_in+1:K*d+k*n_in) = -lambda;
    if nargout > 2
        if all(sum(abs(repmat(lambda,1,n_in) - repmat(lambda,1,n_in)') < 1e-5)==1) %the eigenvalue is not repeated
            V = V(:,ind);
            for i=in
                i_s = K + n_in*K + (k-1)*(n_in/2*(n_in+1)+n_out*n_in);
                dLambda_C = V(:,i)*V(:,i)';
                for j=in
                    dc(i_s+1:i_s+n_in-j+1,K*d+(k-1)*n_in+i) = -dLambda_C(j:n_in,j)*2;
                    i_s = i_s+n_in-j+1;
                end
            end
        else %computing using finite difference
            i_s = K + d*K + (k-1)*(n_in/2*(n_in+1)+n_out*n_in);
            for i=in
                for j=i:n_in
                    Btmp = Sigma(in,in,k);
                    if i==j
                        Btmp(i,i) = Btmp(i,i) + 2*delta;
                    else
                        Btmp(j,i) = Btmp(j,i) + delta;
                        Btmp(i,j) = Btmp(i,j) + delta;
                    end
                    lambda2 = sort(eig(Btmp));
                    dlambda = (lambda2-lambda)/delta;
                    i_s = i_s + 1;
                    dc(i_s,K*d+(k-1)*n_in+1:K*d+k*n_in) = -dlambda;
                end
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors Mu Sigma A L_x] = shape_DS(p,K,in,out)
% transforming the column of parameters into Priors, Mu, and Sigma
n_in = length(in);
n_out = length(out);
L_x = zeros(n_in,n_in,K);
Mu = zeros(n_in+n_out,K);
Sigma = zeros(n_in+n_out,n_in+n_out,K);
A = zeros(n_out,n_in,K);

if K==1
    Priors = 1;
else
    Priors = 1./(1+exp(-p(1:K)));
end


Mu(in,:) = reshape(p(K+1:K+n_in*K),n_in,K);

i_c = K+n_in*K+1;
for k=1:K
    for i=1:n_in
        L_x(i:n_in,i,k) = p(i_c:i_c+n_in-i);
        i_c = i_c + n_in - i + 1;
    end
    A(:,:,k) = reshape(p(i_c:i_c+n_out*n_in-1),n_out,n_in);
    i_c = i_c + n_out*n_in;
%     Sigma(1:d,1:d,k) = L_x(:,:,k)*L_x(:,:,k)';
    Sigma(in,in,k) = L_x(:,:,k) + L_x(:,:,k)';
    [v lambda] = eig(Sigma(in,in,k));
    if any(diag(lambda)< 0)
        lambda(lambda<=0) = 1e-6;
        Sigma(in,in,k) = v'*lambda*v;
    end

    Sigma(out,in,k) = A(:,:,k)*Sigma(in,in,k);
    Mu(out,k) = A(:,:,k)*Mu(in,k);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p0 = GMM_2_Parameters(Priors,Mu,Sigma,K,in,out)
% transforming optimization parameters into a column vector
p0 = [-log(1./Priors(:)-1);reshape(Mu(in,:),[],1)];

for k=1:K
    tmp_mat = Sigma(in,in,k) - diag(diag(Sigma(in,in,k))/2);
%     tmp_mat = chol(Sigma(1:d,1:d,k))';
    for i = in
        p0 = [p0;tmp_mat(i:in(end),i)]; %#ok<*AGROW>
    end
    A_tmp = Sigma(out,in,k)/Sigma(in,in,k);
    p0 = [p0;reshape(A_tmp,[],1)];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors Mu Sigma] = Parameters_2_GMM(popt,K,in,out,options)
% transforming the column of parameters into Priors, Mu, and Sigma
[Priors Mu Sigma tmp] = shape_DS(popt,K,in,out);
if options.normalization
    for k=1:K
        Sigma(:,:,k) = options.Wn\Sigma(:,:,k)/options.Wn;
        Mu(in,k) = options.Wn(in,in)\Mu(in,k);
        Mu(out,k) = Sigma(out,in,k)/Sigma(in,in,k)*Mu(in,k);
        Sigma(in,out,k) = Sigma(out,in,k)';
    end
end