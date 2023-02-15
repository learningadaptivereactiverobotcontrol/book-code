function [Vxf] = learn_wsaqf_lags(Data, varargin)
%%%%%%%%%% Initialization Parameters %%%%%%%%%%
Vxf0.d = size(Data,1)/2;
% Vxf0.w = 1e-4; %A positive scalar weight regulating the priority between the 
               %two objectives of the opitmization. Please refer to the
               %page 7 of the paper for further information.
Vxf0.w = 1e-4;               

% A set of options that will be passed to the solver. Please type 
% 'doc preprocess_demos' in the MATLAB command window to get detailed
% information about other possible options.
options.tol_mat_bias = 10^-1; % a very small positive scalar to avoid
                              % having a zero eigen value in matrices P^l [default: 10^-15]
                              
options.display = 1;          % An option to control whether the algorithm
                              % displays the output of each iterations [default: true]
                              
options.tol_stopping=10^-10;  % A small positive scalar defining the stoppping
                              % tolerance for the optimization solver [default: 10^-10]

options.max_iter = 1000;       % Maximum number of iteration for the solver [default: i_max=1000]

options.optimizePriors = false;% This is an added feature that is not reported in the paper. In fact
                              % the new CLFDM model now allows to add a prior weight to each quadratic
                              % energy term. IF optimizePriors sets to false, unifrom weight is considered;
                              % otherwise, it will be optimized by the solver.
                              
options.upperBoundEigenValue = true; %This is also another added feature that is impelemnted recently.
                                     %When set to true, it forces the sum of eigenvalues of each P^l 
                                     %matrix to be equal one. 


%%%%%%%%%% Initial Guess for WSAQF Parameters %%%%%%%%%% 
if nargin > 1
    shifts = varargin{1}
    Vxf0.L = size(shifts,2);
    Vxf0.Mu = [zeros(Vxf0.d,1) shifts];    
    Vxf0.Priors = ones(Vxf0.L+1,1);
    
    % Fill out initial P matrices
    for l=1:Vxf0.L+1
        Vxf0.P(:,:,l) = eye(Vxf0.d);
    end    
    
    % If prior P_g is given
    P_init = varargin{2};
    Vxf0.P(:,:,1) = P_init;

    options.beta_eps = 0;    
    
    % Solving Nadia's Optimization
    Vxf = my_learnEnergy2(Vxf0,Data,options);
else
    Vxf0.L = 0; % This has to be set somewhere  
    Vxf0.Mu = zeros(Vxf0.d,Vxf0.L+1);     
    Vxf0.Priors = ones(Vxf0.L+1,1);
    Vxf0.Priors = Vxf0.Priors/sum(Vxf0.Priors);
    for l=1:Vxf0.L+1
        Vxf0.P(:,:,l) = eye(Vxf0.d);
    end
    % Solving Mohi's Optimization
    Vxf = learnEnergy(Vxf0,Data,options);    
end



end