% *Linear semidefinite programming with SeDuMi or SDPA input*
% 
% Assume that a linear SDP problem is stored in a SeDuMi or SDPA input file, 
% for instance |arch0.mat| or |arch0.dat-s|, respectively, stored in 
% directory |datafiles|. All the user has to do to solve the problem is to
% call
%
[x] = solve_sedumi('datafiles/sedumi-arch0.mat');
%
% or
% 
[x] = solve_sdpa('datafiles/arch0.dat-s');

% These routines, in turn, call the following sequences of commands
%
% SeDuMi input:
%
 pen = sed2pen('datafiles/sedumi-arch0.mat');
 bmidata=pen2bmi(pen);
 penm=bmi_define(bmidata);
 prob=penlab(penm);
 prob.solve();
 x = prob.x;
%
% SDPA input:
%
 sdpdata=readsdpa('../../datafiles/control1.dat-s');
 penm=sdp_define(sdpdata)
 prob=penlab(penm);
 prob.solve();
 x = prob.x;

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013