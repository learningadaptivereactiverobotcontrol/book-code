function [logPrSigma, cholInvSigma] = logPr_InvWishart( invSigma, nu, Lambda )
% Compute prob. of drawing inv( invSigma ) from InvWishart distr.
%       given hyperparameters PP
% INPUT:
%   invSigma : D x D x N, N iid draws from the distribution param'd by PP
% PP has fields
%   PP.degFree : degrees of freedom, > D-1 (Murphy's v, Wiki's v)
%   PP.ScaleMatrix : D x D psd matrix  (Murphy's S, Wiki's Psi )
% Reference: 
%  www.cs.ubc.ca/~murphyk/Papers/bayesGauss.pdf (eqs 296-297)
%  http://en.wikipedia.org/wiki/Inverse-Wishart_distribution
% These seem to disagree on the norm constant, but Wiki's appears right
%   Murphy's norm constant Z should be flipped over (Z instead of 1/Z)
%   Verified based on numerical integration (see VerifyLogPrInvWishart.m)

N = size( invSigma, 3 );
D = size( invSigma, 2 );

% v = PP.degFree;
% S  = PP.ScaleMat;

v = nu;
S = Lambda;

logDetS = 2*sum( log( diag( chol(S) ) ) );

% Note that because we have invSigma instead of Sigma
%   we flip the sign of the term with det( Sigma )
if N == 1
    cholInvSigma = chol( invSigma );
    logDetInvSigma = 2*sum( log( diag( cholInvSigma ) ) );
    logPrSigma = -0.5*v*D*log(2) - logMvGamma( 0.5*v, D) ...
              + 0.5*v*logDetS ...
              + 0.5*( v + D+1 )*logDetInvSigma ...
              - 0.5*sum(sum( invSigma.*S)); %trace( S*invSigma );
else
    logNormC = -0.5*v*D*log(2) - logMvGamma( 0.5*v, D) ...
              + 0.5*v*logDetS;
    logDataTerm = -Inf( 1, N );
    for n = 1:N        
        curInvS = invSigma(:,:,n);
        cholInvSigma = chol( curInvS );
        logDetInvSigma = 2*sum( log( diag( cholInvSigma ) ) );
        logDataTerm(n) = 0.5*( v + D+1 )*logDetInvSigma ...
              - 0.5*trace( S*curInvS );
    end
    logPrSigma = logNormC + logDataTerm;
end