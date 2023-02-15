function [sqrtx,sqrtinvx,di] = sample_iwishart(sigma,df,di)
%RANDIWISHART Generate inverse Wishart random matrix
%   W=RANDIWISHART(SIGMA,DF) generates a random matrix W from the inverse
%   Wishart distribution with parameters SIGMA and DF.  The inverse of W
%   has the Wishart distribution with covariance matrix inv(SIGMA) and DF
%   degrees of freedom.
%
%   W=RANDIWISHART(SIGMA,DF,DI) expects DI to be the Cholesky factor of
%   the inverse of SIGMA.
%
%   [W,DI]=RANDIWISHART(SIGMA,DF) returns DI so it can be used again in
%   future calls to RANDIWISHART.

n = size(sigma,1);
if (df<n) % require this to ensure invertibility
   error('randiwish:BadDf',...
         'Degrees of freedom must be no smaller than the dimension of SIGMA.');
end

% Get Cholesky factor for inv(sigma) unless that's already done
if nargin<3
    %     [d,p] = chol(sigma,0);
    %     if p~=0
    %         error('stats:iwishrnd:BadCovariance',...
    %             'Covariance matrix must be symmetric and positive definite.');
    %     end
    d = chol(sigma);
    di = d'\eye(size(d));  % either take inverse here and scale chol of
    %randwishart sample and then take inverse of sample, or take inverse of
    %sample and then scale after w/o the inverse.
end

a = randwishart(df/2,n);
sqrtinvx = sqrt(2)*a*di;
sqrtx = (sqrtinvx\eye(size(sqrtinvx)))';

% x = 2*(a'*a);
% x = x\eye(size(x));
% x = d'*(x*d);

% sqrtx = sqrt(2)*a;
% sqrtx = (sqrtx\eye(size(sqrtx)))';
% sqrtx = sqrtx*d;