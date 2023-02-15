function [dx, data] = fmo_objgrad(x, Y, data)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

nelem = length(Y);
dx = sparse(6*nelem+1,1);
dx(1) = 1;
