function lp = lpnormalinvwish(mu,sigma,mu_0,k_0,v_0,lambda_0)
% function lp = lpnormalinvwish(mu,sigma,mu_0,k_0,v_0,lambda_0)
%
%   returns the log density function evaluated at point my/sigma with
% parameters mu_0,k_0,v_0,lambda_0 conforming to Gelman's notation

% Copyright October, 2006, Brown University, Providence, RI. 
% All Rights Reserved 

% Permission to use, copy, modify, and distribute this software and its
% documentation for any purpose other than its incorporation into a commercial
% product is hereby granted without fee, provided that the above copyright
% notice appear in all copies and that both that copyright notice and this
% permission notice appear in supporting documentation, and that the name of
% Brown University not be used in advertising or publicity pertaining to
% distribution of the software without specific, written prior permission. 

% BROWN UNIVERSITY DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
% INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR ANY
% PARTICULAR PURPOSE. IN NO EVENT SHALL BROWN UNIVERSITY BE LIABLE FOR ANY
% SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
% RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
% CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
% CONNECTION WITH THE USE.

% Author: Frank Wood fwood@cs.brown.edu

lp = fvnlp(mu,mu_0,sigma);

k = size(mu_0,1);

lp = lp -(v_0*k/2*log(2) + k*(k-1)/4*log(pi) + sum(gammaln((v_0+1-1:k)/2)))+v_0/2*log(det(lambda_0))-(v_0+k+1)/2*det(sigma)-1/2*trace(lambda_0*inv(sigma));
