function [penm] = corr_define(kappa,H)
% CORR_DEFINE defines PenLab structure for Example 7.1 from the PENLAB paper,
% nearest correlation matrix with the constrained condition number.
% Both input arguments are optional to redefine the default values.
%
% Call:
% >> penm = corr_define;
% >> problem = penlab(penm);
% >> problem.solve();
% >> eig(problem.Y{1}*problem.x)

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  penm = [];

  penm.probname = 'NCM, Example 7.1 from PENLAB paper';
  penm.comment  = 'a scalar and a full matrix as variables';

  % matrix H
  if (nargin<2)
    H = [1 -0.44 -.2 .81 -.46 -0.05;
      -.44 1 .87 -.38 .81 -.58;
      -.2 .87 1 -.17 .65 -.56;
      .81 -.38 -.17 1 -.37 -.15;
      -.46 .81 .65 -.37 1 .08;
      -.05 -.58 -.56 -.15 .08 1];
  end

  if (nargin<1)
    kappa = 10;
  end

  n = size(H,1);

  % keep the whole structure
  penm.userdata.H = H;
  [dum,idiag] = svec2(H); clear dum
  penm.userdata.idiag = idiag;

  % one 'normal' variable
  penm.Nx = 1;
  % one matrix variable
  penm.NY = 1;
  penm.Y{1} = ones(n,n); % to define sparsity structure of Y

  % box constraints on matrix variables
  penm.lbY = [1];
  penm.ubY = [kappa];
  
  % nonlinear constraints
  penm.NgNLN = n;
  penm.lbg = ones(n,1);
  penm.ubg = ones(n,1);

  penm.objfun = @corr_objfun;
  penm.objgrad = @corr_objgrad;
  penm.objhess = @corr_objhess;

  penm.confun = @corr_confun;
  penm.congrad = @corr_congrad;
  penm.conhess = @corr_conhess; 
  
  % starting point
  penm.Yinit{1} = eye(n,n);
  penm.xinit = 1;

