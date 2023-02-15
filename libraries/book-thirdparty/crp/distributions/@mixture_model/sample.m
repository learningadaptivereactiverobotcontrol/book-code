function [samples,classes] = sample(d,cases)
% MIXTURE_MODEL/SAMPLE 
%   [samples,classes] = sample(dist,cases) returns cases number of samples
%   from the mixture model dist. classes contains the class assignments of 
%   the samples

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

% treat the weights as a multinomial to determine which of the mixture
% components to sample from.

if(nargin<2)
    cases = 1;
end

components_to_sample = sample(d.weight,cases);

csc = zeros(length(d.weight),1);
for(i=1:length(d.weight))
    csc(i) = length(find(components_to_sample==i));
end

samples = zeros(dimension(d.component{i}),cases);
classes = zeros(1,cases);

csi = cumsum(csc);
lsi = 1;
for(i=1:length(d.weight))
    samples(:,lsi:lsi+csc(i)-1) = sample(d.component{i},csc(i));
    classes(lsi:lsi+csc(i)-1) = i;
    lsi = lsi+csc(i);
end
