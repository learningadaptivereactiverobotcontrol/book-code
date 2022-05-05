function [der_val]=getKernelFirstDerivative(x1, x2, lambda, type, der_wrt)
% This function calculates the kernel gradient at the given points
%
%   Inputs ----------------------------------------------------------------
%   o x1      :  D x 1 vector x_i
%   o x2      :  D x 1 vector x_j
%   o lambda  :  Inverse kernel width ( 1/*2*sigma*sigma )
%   o type    :  Kernel type. 'rbf' or 'poly'
%   o der_wrt :  1 or 2. Calculate derivative w.r.t x1 or x2 respectively.
%
%   Outputs ---------------------------------------------------------------
%   o der_val :  D x 1 gradient vector
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

if(isscalar(lambda))
   lambda = lambda*ones(size(x1,1),1);  
end
if~(der_wrt ==1 || der_wrt ==2)
   disp('der_wrt only accepts [1] or [2]');
   der_val = zeros(length(x1),1);
   return;
end
switch type
    
    case 'poly'
        if(der_wrt == 1)
            der_val = lambda*(x1'*x2+1)^(lambda-1)*x2;
        else
            der_val = lambda*(x1'*x2+1)^(lambda-1)*x1;
        end
    case 'rbf'
        if(der_wrt == 1)
            der_val = -2*exp(-(lambda.*(x1-x2))'*(x1-x2))*(lambda.*(x1-x2));
        else
            der_val = -2*exp(-(lambda.*(x1-x2))'*(x1-x2))*(lambda.*(x2-x1));
        end
    case 'rbf-linear'
        if(der_wrt == 1)
            der_val = -2*lambda.rbf*exp(-lambda.rbf*norm(x1-x2)^2)*(x1-x2) +x2*lambda.linear;
        else
            der_val = -2*lambda.rbf*exp(-lambda.rbf*norm(x1-x2)^2)*(x2-x1) + x1*lambda.linear;
        end
    otherwise
        disp('Kernel type not recognised in getKij');
        der_val=zeros(length(x1),1);
end

