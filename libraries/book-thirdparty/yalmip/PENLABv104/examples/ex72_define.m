function [penm] = ex72_define()
% Example 7.2, problem (17) from PENNON user's guide
%

  penm = [];

  penm.probname= 'Example 7.2 from PENNON User''s Guide';
  penm.comment = 'a scalar and a 6x6 full matrix as variables';

  % matrix H
  H = [1 -0.44 -.2 .81 -.46 -0.05;
    -.44 1 .87 -.38 .81 -.58;
    -.2 .87 1 -.17 .65 -.56;
    .81 -.38 -.17 1 -.37 -.15;
    -.46 .81 .65 -.37 1 .08;
    -.05 -.58 -.56 -.15 .08 1];

  % keep the whole structure
  penm.userdata.H = H;
  [dum,idiag] = packmat(H); clear dum
  penm.userdata.idiag = idiag;

  % one 'normal' variable
  penm.Nx=1;
  % one matrix variable
  penm.NY=1;
  penm.Y{1}=H; %to define sparsity structure of Y

  % box constraints on matrix variables
  penm.lbY = [1];
  penm.ubY = [10];
  
  % nonlinear constraints
  penm.NgNLN=6;
  penm.lbg = [1;1;1;1;1;1];
  penm.ubg = [1;1;1;1;1;1];

  penm.objfun = @ex72_objfun;
  penm.objgrad = @ex72_objgrad;
  penm.objhess = @ex72_objhess;

  penm.confun = @ex72_confun;
  penm.congrad = @ex72_congrad;
  penm.conhess = @ex72_conhess; 
  
  penm.Yinit{1}=eye(6);
  penm.xinit = 1;
