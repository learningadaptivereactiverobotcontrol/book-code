function [ class ] = calculateClass( model, query_point )
% This function calculates the value of the classifier function at the
% given query point
%
%   Inputs ----------------------------------------------------------------
%   o model        :  The SVM object (struct)
%   o query_point  :  Vector of length D (dimension of state space)
%
%   Outputs ---------------------------------------------------------------
%   o class        :  Predicted class
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


class = sign(calculateGamma(model,query_point));

end

