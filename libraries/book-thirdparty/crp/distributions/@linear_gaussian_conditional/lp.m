function p = lp(d,y,x)
% LINEAR_GAUSSIAN_CONDITIONAL/P 
%       p = lp(d,y,x)

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

if(size(x,1)==1)
    xr = repmat(x,size(y,1),1);
    p = lp(d.gaussian,y-xr*d.A);
elseif(size(y,1)==1)
    yr = repmat(y,size(x,1),1);
    p = lp(d.gaussian,yr-x*d.A);
elseif(size(x,1)==size(y,1))
    p = lp(d.gaussian,y-x*d.A);
else
    error('Either the dimensions of y and x much match or a single y or x may be passed');
end
