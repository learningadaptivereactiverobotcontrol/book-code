function [x] = solve_sdpa(filename)
% SOLVE_SDPA solves linear semidefinite programming with SDPA input
% 
% Assumes that a linear SDP problem is stored in an SDPA input file, for
% instance |arch0.dat-s| stored in directory |datafiles|. 

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

sdpdata=readsdpa(filename);
penm=sdp_define(sdpdata);
prob=penlab(penm);
%prob.opts.outer_stop_limit=1e-2;
prob.opts.kkt_stop_limit=1e-3;
%prob.opts.mlt_update=0.7;
prob.solve();
x=prob.x;