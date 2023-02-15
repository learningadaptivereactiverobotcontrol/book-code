function [A_c, b_c, P] = optimize_lpv_ds_from_data(Data, attractor, ctr_type, gmm, varargin)
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
% Dimensionality
[M, N] = size(Data);
M = M/2;

% Positions and Velocity Trajectories
Xi_ref = Data(1:M,:);
Xi_ref_dot = Data(M+1:end,:);

% Define Optimization Variables
sdp_options = []; Constraints = [];
epsilon = 0.001;
init_cvx = 0;

% Define DS Variables
K = length(gmm.Priors);
A_c = zeros(M,M,K);
b_c = zeros(M,K);

% Define solver for type of constraints
switch ctr_type
    case 0
        % 'sedumi': semidefinite programming solver for convex problems
        sdp_options = sdpsettings('solver','sedumi','verbose', 1, 'debug', 1);
    
    case 1
        % 'penlab': Nonlinear semidefinite programming solver
        sdp_options = sdpsettings('solver','penlab','verbose', 1,'usex0',1, 'debug', 1);
        P_var = sdpvar(M, M, 'symmetric','real');
        Constraints = [Constraints, P_var >  eye(M,M)];
        assign(P_var,eye(M));
        init_cvx = varargin{2};
        
    case 2
        % 'penlab': Nonlinear semidefinite programming solver
        sdp_options = sdpsettings('solver','penlab','verbose', 1,'usex0',1, 'debug', 1);
        P = varargin{1};
        init_cvx = varargin{2};
end

if init_cvx
    % Solve Problem with Convex constraints first to get A's
    fprintf('Solving Optimization Problem with Convex Constraints for Non-Convex Initialization...\n');
    [A0, b0] = optimize_lpv_ds_from_data(Data, attractor, 0, gmm);
end

% Posterior Probabilities per local model
h_k = posterior_probs_gmm(Xi_ref,gmm,'norm');

% Define Constraints and Assign Initial Values
for k = 1:K    
    A_vars{k} = sdpvar(M, M, 'full','real');       
    b_vars{k} = sdpvar(M, 1, 'full');
    Q_vars{k} = sdpvar(M, M,'symmetric','real');       
       
    % Assign Initial Parameters
    if init_cvx       
        assign(A_vars{k},A0(:,:,k));
        assign(b_vars{k},b0(:,k));        
    else
        assign(A_vars{k},-eye(M));
        assign(b_vars{k},-eye(M)*attractor);
    end
    
    % Define Constraints
    switch ctr_type
        case 0 %: convex
            Constraints = [Constraints transpose(A_vars{k}) + A_vars{k} <= -epsilon*eye(M,M)]; 
            Constraints = [Constraints b_vars{k} == -A_vars{k}*attractor];
        
        case 1 %: non-convex, unknown P (Dataset assumed to be shifted to the origin)            
            Constraints = [Constraints, transpose(A_vars{k})*P_var + P_var*A_vars{k} <= -epsilon*eye(M)];                      

        case 2 %: non-convex with given P
            
            % Option 2: Less Strict and converges faster most of the times                      
            Constraints = [Constraints, transpose(A_vars{k})*P + P*A_vars{k} == Q_vars{k}];                        
            Constraints = [Constraints, Q_vars{k} <= -epsilon*eye(M)];                        
            Constraints = [Constraints, b_vars{k} == -A_vars{k}*attractor];                                 
            assign(Q_vars{k},-eye(M));
            
    end
end

% Calculate our estimated velocities caused by each local behavior
Xi_d_dot_c_raw = sdpvar(M,N,K, 'full');%zeros(size(Qd));
for k = 1:K
    h_K = repmat(h_k(k,:),[M 1]);
    if ctr_type == 1
        f_k = A_vars{k}*Xi_ref;     
    else
        f_k = A_vars{k}*Xi_ref + repmat(b_vars{k},[1 N]);
    end
    Xi_d_dot_c_raw(:,:,k) = h_K.*f_k;
end

% Sum each of the local behaviors to generate the overall behavior at
% each point
Xi_d_dot = sdpvar(M, N, 'full');
Xi_d_dot = reshape(sum(Xi_d_dot_c_raw,3),[M N]);

% Then calculate the difference between approximated velocities
% and the demonstated ones for A
Xi_dot_error = Xi_d_dot - Xi_ref_dot;

% Defining Objective Function depending on constraints
if ctr_type == 0
    Xi_dot_total_error = sdpvar(1,1); Xi_dot_total_error(1,1) = 0;
    for n = 1:N
        Xi_dot_total_error = Xi_dot_total_error + norm(Xi_dot_error(:, n));
    end
    Objective = Xi_dot_total_error;
else
    Aux_var     = sdpvar(M,length(Xi_dot_error));
    Objective   = sum((sum(Aux_var.^2)));
    Constraints = [Constraints, Aux_var == Xi_dot_error];
end

% Solve optimization problem
sol = optimize(Constraints, Objective, sdp_options)
if sol.problem ~= 0
    yalmiperror(sol.problem);
end

for k = 1:K
    A_c(:,:,k) = value(A_vars{k});
    b_c(:,k)   = value(b_vars{k});
end

switch ctr_type 
    case 0 
        P = eye(M);
    case 1
        P   = value(P_var);
        b_c = zeros(M,K);
end

sol.info
check(Constraints)
fprintf('Total error: %2.2f\nComputation Time: %2.2f\n', value(Objective),sol.solvertime);


%%%% FOR DEBUGGING: Check Negative-Definite Constraint %%%%
if ctr_type == 0
    constr_violations = zeros(1,K);
    for k=1:K
        A_t = A_c(:,:,k) + A_c(:,:,k)';
        constr_violations(1,k) = sum(eig(A_t) > 0); % sufficient
    end
    % Check Constraint Violation
    if sum(constr_violations) > 0
        warning(sprintf('Strict System Matrix Constraints are NOT met..'))
    else
        fprintf('All Sufficient System Matrix Constraints are met..\n')
    end
else
    suff_constr_violations = zeros(1,K);
    for k=1:K
        Pg_A =  A_c(:,:,k)'*P + P*A_c(:,:,k);
        suff_constr_violations(1,k) = sum(eig(Pg_A + Pg_A') > 0); % strict
    end
    % Check Constraint Violation
    if sum(suff_constr_violations) > 0
        warning(sprintf('Sufficient System Matrix Constraints are NOT met..'))
    else
        fprintf('All Sufficient System Matrix Constraints are met..\n')
    end
end

end

