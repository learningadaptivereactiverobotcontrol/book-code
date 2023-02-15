function logPr = logPr_Gaussian( X, mu, invSigma)
% Each row of X is an independent draw from Normal( mu, invSigma )

D = size( invSigma,1);
LOG_2PI = 1.837877066409345;

cholInvSigma = chol( invSigma );
logDetInvSigma = 2*sum( log( diag( cholInvSigma ) ) );

XdiffMu = bsxfun( @minus, X, mu );
U = XdiffMu'*cholInvSigma';
logPr = -0.5*D*LOG_2PI +  0.5*logDetInvSigma  - 0.5*sum(U.^2, 2);