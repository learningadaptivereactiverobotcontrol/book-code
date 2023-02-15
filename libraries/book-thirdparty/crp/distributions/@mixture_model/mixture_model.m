function mm = mixture_model(varargin)
%MIXTURE_MODEL mixture model class constructor.
%   d = MIXTURE_MODEL(dist1, dist2, dist3, ..., distn) creates a mixture
%   distribution with n component densities

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

bc = distribution();
   d.weight = multinomial();
    d.component = {};
    d.num_components = 0;
if(nargin==0)
    d.weight = multinomial();
    d.component = {};
    d.num_components = 0;

else
    if(mod(nargin,2)~=0)
        error('Number of arguments should be even (weight/dist), (weight/dist), etc.')
    end
    %     d.weight = multinomial(ones(nargin/2,1)/(nargin/2));
    dw = ones(nargin/2,1)/(nargin/2);
    d.component = cell(nargin/2,1);
    d.num_components = nargin/2;
    for(i=1:d.num_components)
        if(~isnumeric(varargin{i*2-1}))
            error(sprintf('Argument # %d isn''t a numeric weight',i*2-1));
        end
        if(~isa(varargin{i*2},'distribution'))
            error(sprintf('Argument # %d isn''t a distribution',i*2));
        end
        dw(i) = varargin{i*2-1};
        d.component{i} = varargin{i*2};

    end
    d.weight = multinomial(dw);

end

mm = class(d,'mixture_model',bc);
