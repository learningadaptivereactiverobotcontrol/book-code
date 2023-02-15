function [f,x] = nlp_solve(nlname)
% NLP_SOLVE solves AMPL based NLP problem by the old AMPL interface
% nlname ... name of ampl nl-file
%
% the current interface reformulates the problem into
%    min   f(x)
%    s.t.  g(x)<=0
%          h(x)=0
% where first N_BOUNDS of g(x) are bounds constraints
% Don't know which ones are nonlinear --> declare all as NLN
% in the order inequalitites, equalitites. Bounds will try to 
% detect extra.

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  if (nargin<1 || isempty(nlname))
    error('Specify the name of an Ampl nl-file.');
  end

penm=nlp_define(nlname);
prob=penlab(penm);
prob.solve();

f = prob.objx;
x = prob.x;