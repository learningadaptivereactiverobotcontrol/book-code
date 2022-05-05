function [ gradient ] = calculateGammaDerivative( model, query_point )
% This function calculates the gradient of the classifier function at the
% given point
%
%   Inputs ----------------------------------------------------------------
%   o model        :  The SVM object (struct)
%   o query_point  :  Vector of length D (dimension of state space)
%
%   Outputs ---------------------------------------------------------------
%   o gradient     :  D x 1 gradient vector.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

gradient = zeros(size(query_point));
type   = 'rbf';
sigma  = model.sigma;
lambda = 1 / (2*sigma*sigma);

for i=1:model.nSV 
    gradient  = gradient + model.yalphas(i)*getKernelFirstDerivative(query_point, model.SVs(:,i), lambda, type, 1);
end

end

