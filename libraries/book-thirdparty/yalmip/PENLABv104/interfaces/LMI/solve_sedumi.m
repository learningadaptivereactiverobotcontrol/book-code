function [x] = solve_sedumi(filename)
% SOLVE_SEDUMI solves linear semidefinite programming with SeDuMi input
% 
% Assumes that a linear SDP problem is stored in an SeDuMi input file, for
% instance |arch0.dat-s| stored in directory |datafiles|. 

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

pen = sed2pen(filename);
bmidata=pen2bmi(pen);
penm=bmi_define(bmidata);
prob=penlab(penm);
%prob.opts.outer_stop_limit=1e-2;
%prob.opts.kkt_stop_limit=1e-3;
%prob.opts.mlt_update=0.7;
prob.solve();
x=prob.x;