function [Vxf J] = learnEnergy(Vxf0,Data,options)
%
% CLFDM software package: version 1.0, released on 23 March 2015
%
% This function builds an estimate of a Lyapunov (energy) function from
% demonstrations.
%
% Syntax:
%       [Vxf J] = learnEnergy(Vxf0,Data,options)
%
% to also pass a structure of desired options.
%
% Important NOTE: Both the demonstration data, and the model estimation
% should be in the target frame of reference. In other words, this codes
% assumes that the target is at the origin!
%
% Inputs -----------------------------------------------------------------
%
%   o Vxf_0:     A structure variable representing the initial guess for the
%                energy function. Please refer to the Output variable for
%                further details about its fields.
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
%                            having a zero eigen value in matrices P^l [default: 10^-15]
%
%       - .tol_stopping:     A small positive scalar defining the stoppping
%                            tolerance for the optimization solver [default: 10^-10]
%
%       - .i_max:            maximum number of iteration for the solver [default: i_max=1000]
%
%       - .display:          An option to control whether the algorithm
%                            displays the output of each iterations [default: true]
%
%       - .optimizePriors    This is an added feature that is not reported in the paper. In fact
%                            the new CLFDM model now allows to add a prior weight to each quadratic
%                            energy term. IF optimizePriors sets to false, unifrom weight is considered;
%                            otherwise, it will be optimized by the sovler. [default: true]
%                              
%       - .upperBoundEigenValue     This is also another added feature that is impelemnted recently.
%                                   When set to true, it forces the sum of eigenvalues of each P^l 
%                                   matrix to be equal one. [default: true]
%
%
% Outputs ----------------------------------------------------------------
%
%   o Vxf:      A structure variable representing the energy function. It
%               is composed of the following fields:
%
%       - .d:       Dimension of the state space, d>0.
%    
%       - .L:       The number of asymmetric quadratic components L>=0.
%    
%       - .Priors:  1 x K array representing the prior weight of each energy
%                  component. Prioris are positive scalars between 0 and 1.
%    
%       - .Mu:      Each Mu(:,i) is a vector of R^d and represent the center of
%                  the energy component i. Note that by construction Mu(:,1)=0!
%    
%       - .P:       Each P(:,:,i), i=1:L+1, is a d x d positive definite matrix.
%                  P(:,:,1) corresponds to the symmetric energy component P^0.
%                  P^1 to P^{L+1} are asymmetric quadratic terms. Note that the
%                  matrices P^i are not necessarily symmetric.
%    
%       - .w:      A positive scalar weight regulating the priority between the 
%                  two objectives of the opitmization. Please refer to the
%                  page 7 of the paper for further information.
%    
%       - .SOS:    This is an internal variable, and is automatically set to false
%
%   o J:      The value of the objective function at the optimized point
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         Copyright (c) 2014 Mohammad Khansari, LASA Lab, EPFL,       %%%
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
% S.M. Khansari-Zadeh and A. Billard (2014), "Learning Control Lyapunov Function
% to Ensure Stability of Dynamical System-based Robot Reaching Motions." 
% Robotics and Autonomous Systems, vol. 62, num 6, p. 752-765.
%
% To get latest update of the software please visit
%                          http://cs.stanford.edu/people/khansari/
%
% Please send your feedbacks or questions to:
%                          khansari_at_cs.stanford.edu

%% initializing ...

% parsing options
if isempty(options)
    options = check_options();
else
    options = check_options(options); % Checking the given options, and add ones are not defined.
end

d = size(Data,1)/2; %dimension of the model
x = Data(1:d,:);
xd = Data(d+1:2*d,:);
Vxf0.SOS = false;

%% Optimization
%transforming the Lyapunov model into a vector of optimization parameters
if Vxf0.SOS
%     p0 = zeros(d*Vxf0.n,d*Vxf0.n);
%     p0(d+1:2*d,d+1:2*d) = eye(d);
    p0 = randn(d*Vxf0.n,d*Vxf0.n);
    p0 = p0*p0';
    p0 = p0(:);
    Vxf0.L = -1; %to distinguish sos from other methods
else
    for l = 0:Vxf0.L
        Vxf0.P(:,:,l+1) = Vxf0.P(:,:,l+1)\eye(d);
    end
    
    %in order to set the first component to be the closest Gaussian to origin
    [~, ind] = sort(Mat_Vec_Norm(Vxf0.Mu),'ascend');
    Vxf0.Mu = Vxf0.Mu(:,ind);
    Vxf0.P  = Vxf0.P(:,:,ind);
    p0 = GMM_2_Parameters(Vxf0,options);
end

obj_handle = @(p) obj(p,x,xd,d,Vxf0.L,Vxf0.w,options);
ctr_handle = @(p) ctr_eigenvalue(p,d,Vxf0.L,options);

% Running the optimization
if options.display
    str = 'iter';
else
    str = 'off';
end

% Options for NLP Solvers
optNLP = optimset( 'Algorithm', 'interior-point', 'LargeScale', 'off',...
    'GradObj', 'off', 'GradConstr', 'off', 'DerivativeCheck', 'on', ...
    'Display', 'iter', 'TolX', options.tol_stopping, 'TolFun', options.tol_stopping, 'TolCon', 1e-12, ...
    'MaxFunEval', 200000, 'MaxIter', options.max_iter, 'DiffMinChange', ...
    1e-4, 'Hessian','off','display',str);

% Solve fully-discretized optimal control problem
[popt J] = fmincon(obj_handle, p0,[],[],[],[],[],[],ctr_handle,optNLP);

if Vxf0.SOS
    Vxf.d = d;
    Vxf.n = Vxf0.n;
    Vxf.P = reshape(popt,Vxf.n*d,Vxf.n*d);
    Vxf.SOS = 1;
    Vxf.p0 = compute_Energy(zeros(d,1),[],Vxf);
    check_constraints(popt,ctr_handle,d,0,options);
else
    % transforming back the optimization parameters into the GMM model
    Vxf = Parameters_2_GMM(popt,d,Vxf0.L,options);
    Vxf.Mu(:,1) = 0;
    Vxf.L = Vxf0.L;
    Vxf.d = Vxf0.d;
    Vxf.w = Vxf0.w;
    check_constraints(popt,ctr_handle,d,Vxf.L,options);
end

sumDet = 0;
for l = 1:Vxf.L+1
    sumDet = sumDet + det(Vxf.P(:,:,l));
end
Vxf.P(:,:,1) = Vxf.P(:,:,1)/sumDet;
Vxf.P(:,:,2:end) = Vxf.P(:,:,2:end)/sqrt(sumDet);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [J, dJ]=obj(p,x,xd,d,L,w,options)
% This function computes the derivative of the likelihood objective function
% w.r.t. optimization parameters.

if L == -1
    Vxf.n = sqrt(length(p)/d^2);
    Vxf.d = d;
    Vxf.P = reshape(p,Vxf.n*d,Vxf.n*d);
    Vxf.SOS = 1;
else
    Vxf = shape_DS(p,d,L,options);
end

[~, Vx] = computeEnergy(x,[],Vxf);
Vdot = sum(Vx.*xd,1); %derivative of J w.r.t. xd
norm_Vx = sqrt(sum(Vx.*Vx,1));
norm_xd = sqrt(sum(xd.*xd,1));
J = Vdot./(norm_Vx.*norm_xd);
% J = Vdot./(norm_Vx);
J(norm_xd==0) = 0;
J(norm_Vx==0) = 0;
J(Vdot>0) =    J(Vdot>0).^2;
J(Vdot<0) = -w*J(Vdot<0).^2;
J = sum(J);
dJ = [];



function [c ceq dc dceq]=ctr_eigenvalue(p,d,L,options)
% This function computes the derivative of the constrains w.r.t.
% optimization parameters.

if L == -1 %SOS
    Vxf.d = d;
    Vxf.n = sqrt(length(p)/d^2);
    Vxf.P = reshape(p,Vxf.n*d,Vxf.n*d);
    Vxf.SOS = 1;
    c  = zeros(Vxf.n*d,1);
    ceq = [];
else
    Vxf = shape_DS(p,d,L,options);
    if L > 0
        c  = zeros((L+1)*d+(L+1)*options.optimizePriors,1); %+options.variableSwitch
        if options.upperBoundEigenValue
            ceq = zeros(L+1,1);
        else
            ceq = [];%zeros(L+1,1);
        end
    else
        c  = zeros(d,1);
        ceq = Vxf.P(:)'*Vxf.P(:)-2;
    end
end
dc = [];
dceq = [];

if L == -1 %SOS
    c = -eig(Vxf.P + Vxf.P' - eye(Vxf.n*d)*options.tol_mat_bias);
else
%     ceq = 1;
    for k = 0:L
        lambda = eig(Vxf.P(:,:,k+1) + Vxf.P(:,:,k+1)')/2;
        c(k*d+1:(k+1)*d) = -lambda + options.tol_mat_bias;
        if options.upperBoundEigenValue
            ceq(k+1) = 1.0 - sum(lambda); % + Vxf.P(:,:,k+1)'
        end
%         ceq(k+1) = 2.0 - sum(sum(Vxf.P(:,:,k+1).^2));
    end
end
if L > 0 && options.optimizePriors
    c((L+1)*d+1:(L+1)*d+L+1) = -Vxf.Priors;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Vxf = shape_DS(p,d,L,options)
% transforming the column of parameters into Priors, Mu, and P
P = zeros(d,d,L+1);

if L == 0
    Priors = 1;
    Mu = zeros(d,1);
    i_c = 1;
else
    if options.optimizePriors
        Priors = p(1:L+1);
        i_c = L+1;
    else
        Priors = ones(L+1,1);i_c = 0;
%         Priors = ones(L+1,1);i_c = L+1;
    end
    Priors = Priors/sum(Priors);
    Mu = [zeros(d,1) reshape(p(i_c+(1:d*L)),d,L)];
    i_c = i_c+d*L+1;
end

for k = 0:L
    P(:,:,k+1) = reshape(p(i_c+k*d^2:i_c+(k+1)*d^2-1),d,d);% + eye(d)*options.tol_mat_bias;
%     for i=1:d
%         P(i:d,i,k) = p(i_c:i_c+d-i);
%         i_c = i_c + d - i + 1;
%     end
%     P(:,:,k) = P(:,:,k) + P(:,:,k)' + eye(d)*options.tol_mat_bias;
end

Vxf.Priors = Priors;
Vxf.Mu = Mu;
Vxf.P = P;
Vxf.SOS = 0;
% if options.variableSwitch
%     Vxf.x_sw = p(end);
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p0 = GMM_2_Parameters(Vxf,options)
% transforming optimization parameters into a column vector
d = Vxf.d;
if Vxf.L > 0
    if options.optimizePriors
        p0 = [Vxf.Priors(:);reshape(Vxf.Mu(:,2:end),Vxf.L*d,1)];
    else
        p0 = reshape(Vxf.Mu(:,2:end),Vxf.L*d,1);
    end
else
    p0=[];
end
for k = 0:Vxf.L
    p0 = [p0;reshape(Vxf.P(:,:,k+1),d^2,1)]; %#ok<*AGROW>
%     tmp_mat = Vxf.P(:,:,k) - diag(diag(Vxf.P(:,:,k))/2);
%     for i = 1:d
%         p0 = [p0;tmp_mat(i:d,i)]; %#ok<*AGROW>
%     end
end
% if options.variableSwitch
%     p0 = [p0;Vxf.x_sw];
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Vxf = Parameters_2_GMM(popt,d,L,options)
% transforming the column of parameters into Priors, Mu, and P
Vxf = shape_DS(popt,d,L,options);

% if options.normalization
%     for k=1:K
%         P(:,:,k) = options.Wn\P(:,:,k)/options.Wn;
%         Mu(in,k) = options.Wn(in,in)\Mu(in,k);
%         Mu(out,k) = P(out,in,k)/P(in,in,k)*Mu(in,k);
%         P(in,out,k) = P(out,in,k)';
%     end
% end


function options = check_options(varargin)
% parsing options
if ~isempty(varargin)
    options = varargin{1};
end
if isempty(varargin) || ~isfield(options,'tol_mat_bias')
    options.tol_mat_bias = 10^(-15);
end
if ~isfield(options,'tol_stopping')
    options.tol_stopping = 10^-10;
end
if ~isfield(options,'max_iter')
    options.max_iter = 1000;
end
if ~isfield(options,'display')
    options.display = 1;
else 
    options.display = options.display > 0;
end
if ~isfield(options,'optimizePriors')
    options.optimizePriors = true;
else 
    options.optimizePriors = options.optimizePriors > 0;
end
if ~isfield(options,'upperBoundEigenValue')
    options.upperBoundEigenValue = true;
else 
    options.upperBoundEigenValue = options.upperBoundEigenValue > 0;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function check_constraints(p,ctr_handle,d,L,options)
% checking if every thing goes well. Sometimes if the parameter
% 'options.cons_penalty' is not big enough, the constrains may be violated.
% Then this function notifies the user to increase 'options.cons_penalty'.

c = -ctr_handle(p);

if L > 0
    c_P = reshape(c(1:L*d),d,L)';
else
    c_P = c;
end

[i,~] = find(c_P<=0);
bool_success = true;
if ~isempty(i)
    i = sort(i);
    err = [i(:) c_P(i,:)];
    disp('Error in the constraints on P!')
    disp('Eigenvalues of the P^k that violates the constraints:')
    disp(err)
    bool_success = false;
end

if L>1
    if options.optimizePriors
        c_Priors = c(L*d+1:L*d+L);
        i = find(c_Priors<0);
        if ~isempty(i)
            err = [i(:) c_Priors(i)];
            disp('Error in the constraints on Priors!')
            disp('Values of the Priors that violates the constraints:')
            disp(err)
            bool_success = false;
        end
    end
    
    if length(c)>L*d+L
        c_x_sw = c(L*d+L+1);
        if c_x_sw<=0
            disp('Error in the constraints on x_sw!')
            sprintf('x_sw = %f',c_x_sw)
            bool_success = false;
        end
    end
end

if bool_success
    disp('Optimization finished successfully.')
    disp(' ')
    disp(' ')
else
    disp('Optimization did not reach to an optimal point.')
    disp('Some constraints were slightly violated.')
    disp('Re-run the optimization with different initial guess to handle this issue.')
    disp('Increasing the number of P could be also helpful.')
    disp(' ')
    disp(' ')
end    