function prob = my_gaussPDF(X, Mu, Sigma)
%MY_GAUSSPDF computes the Probability Density Function (PDF) of a
% multivariate Gaussian represented by a mean and covariance matrix.
%
% Inputs -----------------------------------------------------------------
%       o X     : (N x M), a data set with M samples each being of dimension N.
%                          each column corresponds to a datapoint
%       o Mu    : (N x 1), an Nx1 vector corresponding to the mean of the 
%							Gaussian function
%       o Sigma : (N x N), an NxN matrix representing the covariance matrix 
%						   of the Gaussian function
% Outputs ----------------------------------------------------------------
%       o prob  : (1 x M),  a 1xM vector representing the probabilities for each 
%                           M datapoints given Mu and Sigma    
%%

% Auxiliary Variables
[N,M] = size(X);

% Output Variable
prob = zeros(1,M);

% Demean Data
X = bsxfun(@minus, X, Mu);

% Compute Probabilities
% 1) The exponential term is the inner products of the zero-mean data
exp_term = sum((X'*inv(Sigma)).*X', 2)';
% exp_term = sum((X'/Sigma).*X', 2);

% 2) Compute Equation (2)
prob = exp(-0.5*exp_term) / sqrt((2*pi)^N * (abs(det(Sigma))+realmin));
