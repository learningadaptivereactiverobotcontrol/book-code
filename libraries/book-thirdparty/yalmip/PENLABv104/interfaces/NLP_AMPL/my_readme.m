% *Nonlinear optimization with AMPL input*
% 
% Assume that nonlinear optimization problem is defined in and processed by
% <http://www.ampl.com AMPL>, so that we have the corresponding |.nl|
% file, for instance |chain.nl| stored in directory |datafiles|. All the
% user has to do to solve the problem is to call the following three
% commands:
% 
clear all
penm=nlp_define('../../datafiles/rocket800.nl');
prob=penlab(penm);
prob.opts.inner_stop_limit=1e-4;
prob.opts.ldl_pivot=1e-6;
%prob.opts.kkt_stop_limit=1e-8;
%prob.opts.outer_stop_limit=1e-8;
%prob.opts.outlev=2;
%prob.opts.max_inner_iter=500;
%prob.opts.mpenalty_min=1e-8;
%prob.opts.penalty_update=0.3;
prob.solve();

%
% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013