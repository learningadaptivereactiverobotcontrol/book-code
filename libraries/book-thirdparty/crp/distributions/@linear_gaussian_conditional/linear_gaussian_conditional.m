function d = linear_gaussian_conditional(y,x)
%LINEAR_GAUSSIAN_CONDITIONAL linear Gaussian condititional distribution 
%   class constructor. d = LINEAR_GAUSSIAN_CONDITIONAL(y,x) 
%   produces a conditional distribution y ~ N(Ax,W)
%   calls to p= p(d,y,x) return p ~ exp((y-Ax)'sqrt(W)(y-Ax))
%
%       y should be N x y_dim
%       x should ne N x x_dim
%

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

if nargin==2
   d.A = pinv(x)*y;
   
   g = gaussian(y-x*d.A);

   bc = distribution();
   d = class(d,'linear_gaussian_conditional',g,bc);
else
error;
end
