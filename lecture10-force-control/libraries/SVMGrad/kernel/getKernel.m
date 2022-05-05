function [ ker_val ] = getKernel( x1, x2, lambda, type)
% This function calculates the kernel function at the given points
%
%   Inputs ----------------------------------------------------------------
%   o x1     :  D x 1 vector x_i
%   o x2     :  D x 1 vector x_j
%   o lambda :  Inverse kernel width ( 1/*2*sigma*sigma )
%   o type   :  Kernel type. 'rbf' or 'poly'
%
%   Outputs ---------------------------------------------------------------
%   o ker_val :  Scalar output value
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%             Copyright (c) 2012 Ashwini SHUKLA, LASA, EPFL,          %%%
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
%
% To get latest upadate of the software please visit
%                          http://asvm.epfl.ch
%
% Please send your feedbacks or questions to:
%                           ashwini.shukla_at_epfl_dot_ch
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if(isscalar(lambda))
%    lambda = lambda*ones(size(x1,1),1); 
% end
switch type
    
    case 'poly'
        ker_val = (x1'*x2+1)^lambda;
    case 'rbf'
%         ker_val = exp(-(lambda.*(x1-x2))'*(x1-x2));
        ker_val = exp(-lambda.*norm(x1-x2)^2);
    case 'rbf-linear'
        ker_val = exp(-lambda.rbf*norm(x1-x2)^2) + lambda.linear*x1'*x2;
    otherwise
        disp('Kernel type not recognised in getKij');
        ker_val=0;
end

