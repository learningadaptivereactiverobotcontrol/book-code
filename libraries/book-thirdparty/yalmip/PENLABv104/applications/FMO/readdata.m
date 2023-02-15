function par = kobum
%
%   reads the input data and creates the structure
%   "par" of the problem parameters
%
% Matlab coding by Michal Kocvara, University of Birmingham, 2010
% kocvara@maths.bham.ac.uk

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

load exsmall

par.Aik = AA;
par.RHS = F;
par.nelem = nelem;
par.nnod = nnod;
par.nloads = 1;
par.nelx = nelx;
par.nely = nely;
