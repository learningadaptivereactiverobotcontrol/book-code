function [evec, idiag] = svec2(emat)
%
% [evec, idiag] = svec2(emat)
% vectorization of a (generally full) symmetric matrix; 
% upper triangle is vectorized row-wise
% INPUT: evac...symmetric matrix
% OUPUT: evec...upper triangle of emat stored row-wise in a vector
%        idiag...vector oex72f indices of diagonal entries of emat in evec

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

m = size(emat,2);
idiag(1) = 1;
k = m;

evec = zeros(m*(m+1)/2,1);
evec(1:m) = emat(1:m,1);
for j=2:m
    i = idiag(j-1) + k;
    idiag(j) = i;
    k = k-1;
    evec(i:i+k-1) = emat(j:m,j);
end

end