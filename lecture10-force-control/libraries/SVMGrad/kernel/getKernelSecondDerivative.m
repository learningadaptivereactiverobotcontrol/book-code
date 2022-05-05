function [ hes_val ] = getKernelSecondDerivative( x1, x2, lambda, type )
% This function calculates the hessian matrix of the kernel function
%
%   Inputs ----------------------------------------------------------------
%   o x1     :  D x 1 vector x_i
%   o x2     :  D x 1 vector x_j
%   o lambda :  Inverse kernel width ( 1/*2*sigma*sigma )
%   o type   :  Kernel type. 'rbf' or 'poly'
%
%   Outputs ---------------------------------------------------------------
%   o hes_val :  D x D hessian matrix
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

switch type
    
    case 'poly'
        tmp = x1'*x2 + 1;
        hes_val = lambda* tmp^(lambda-2)*( tmp*eye(length(x1)) + (lambda-1)*x2*x1');

    case 'rbf'
        tmp = x1-x2;
        hes_val =2*diag(lambda)*exp(-(lambda.*tmp)'*tmp) * ( eye(length(x1)) - 2*(tmp*tmp')*diag(lambda) );
    case 'rbf-linear'
        tmp = x1-x2;
        hes_val =2*lambda.rbf*exp(-lambda.rbf*norm(tmp)^2) * ( eye(length(x1)) - 2*lambda.rbf*(tmp*tmp') );
    otherwise
        disp('Kernel type not recognised in getKij');
        hes_val=zeros(length(x1),length(x1));
end

