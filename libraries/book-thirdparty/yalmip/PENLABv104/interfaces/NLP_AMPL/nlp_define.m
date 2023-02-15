function [penm] = nlp_define(nlname)
% NLP_DEFINE defines AMPL based NLP problem by the old AMPL interface
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

  [x0, u0, ps] = amplf(nlname,0);

  penm = [];
  % [optional] set problem name/comment (for log files)
  penm.probname = nlname;
  penm.comment = 'AMPL (old) interface';

  penm.userdata = ps;

  penm.xinit=x0;

  % recover box constraints back from g(x)<=0 conversion :-(
  [fx, gx, hx] = amplf(x0,0);
  [fdx, gdx, hdx] = amplf(x0,1);
  % default lower/upper bounds, fill in these which are restricted
  lbx=-Inf(ps.N,1);
  ubx=Inf(ps.N,1);
  [xi,gi,data]=find(gdx(:,1:ps.N_BOUNDS));
  gilb=find(data<0);
  giub=find(data>0);
  % gi(gilb) ... constraint numbers defining lower bounds
  %   gx(gi(gilb)) ... and teir values
  % xi(gilb) ... what elements of x are constrained
  lbx(xi(gilb)) = gx(gi(gilb)) + x0(xi(gilb));
  ubx(xi(giub)) = x0(xi(giub)) - gx(gi(giub));

  penm.Nx = ps.N;
  penm.lbx = lbx;
  penm.ubx = ubx;

  penm.NgNLN = ps.N_CONSTR-ps.N_BOUNDS;  % inequal + equal
  penm.NgLIN = 0;
  penm.lbg = [-Inf(penm.NgNLN-ps.N_EQUAL,1); zeros(ps.N_EQUAL,1)];
  penm.ubg = zeros(penm.NgNLN,1);

  % not ideal, these routines should get the piece computed and 
  % drop the rest
  penm.objfun = @nlp_objfun;
  penm.confun = @nlp_confun;
  penm.objgrad = @nlp_objgrad;
  penm.congrad = @nlp_congrad;
  penm.lagrhess = @nlp_lagrhess;



