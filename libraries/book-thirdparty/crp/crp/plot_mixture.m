function plot_mixture(training_data,class_id,color)
% function plot_mixture(training_data,class_id,color)
%
% plots a mixture model given training_data (Dim x # points), class_id 
% (vector), and a vector of colors.  The defaults color vector is
% 
% color = {'ko', 'ro', 'go', 'bo', 'mo', 'co'};
%
% and if more than 6 classes are present, all classes with higher class_id
% are plotted as 'k.'.  Additionally, only the first two components of
% the training data are plotted

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

if(nargin < 3)
    color = {'ko', 'ro', 'go', 'bo', 'mo', 'co'};
end
clf
ucids = unique(class_id);
hold on
for(i = 1:length(ucids))
    if(ucids(i)<7)
        plot(training_data(1,find(class_id==ucids(i))),...
            training_data(2,find(class_id==ucids(i))),color{ucids(i)})
    else
            plot(training_data(1,find(class_id==ucids(i))),...
                training_data(2,find(class_id==ucids(i))),'k.')
    end
end
hold off
