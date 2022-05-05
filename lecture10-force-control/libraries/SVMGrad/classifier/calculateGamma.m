function [ value ] = calculateGamma( model, query_point )
% This function calculates the value of the decision function at the
% given query point
%
%   Inputs ----------------------------------------------------------------
%   o model        :  The SVM object (struct)
%   o query_point  :  Vector of length D (dimension of state space)
%
%   Outputs ---------------------------------------------------------------
%   o value        :  Scalar value of the function evaluated at this point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

type   = 'rbf';
sigma  = model.sigma;
lambda = 1 / (2*sigma*sigma);

value = 0;
for i=1:model.nSV            
    value  = value + model.yalphas(i)*getKernel(query_point, model.SVs(:,i), lambda, type);
end
value = value + model.b;

end

