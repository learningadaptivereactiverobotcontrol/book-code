% *Nonlinear optimization with AMPL input*
% 
% Assume that nonlinear optimization problem is defined in and processed by
% <http://www.ampl.com AMPL>, so that we have the corresponding |.nl|
% file, for instance |chain.nl| stored in directory |datafiles|. All the
% user has to do to solve the problem is to call the following three
% commands:
% 
penm=nlp_define('../../datafiles/chain100.nl');
prob=penlab(penm);
prob.solve();