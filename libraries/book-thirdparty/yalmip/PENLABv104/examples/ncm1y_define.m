function [penm] = ncm1y_define()
% define penm structure for Nearest Correlation Matrix Example 1
% using matrix variables Y:
%    min_Y sum (y_ij-h_ij)^2
%    s.t.  tr(Y)=1
%          Y >= 0 (pos. semidefinite)

  penm = [];

  penm.probname= 'NCM Example 1 using matrix variables';
  penm.comment = '3 diagonal matrix 3x3 as a matrix variable';

  % matrix H
  %H = [2.2, -1.1, 0; -1.1, 1.9, -1.15; 0, -1.15, 2.1]./6;
  H = [2.2, -1.1, 0; -1.1, 1.9, -1.1; 0, -1.1, 2.1]./6;

  % keep the whole structure
  penm.userdata=H;

  % no 'normal' variables
  penm.Nx=0;
  % one matrix variable of the given 3-diagonal structure (same as H)
  penm.NY=1;
  penm.Y{1}=H;

  penm.Yinit{1}=eye(3);

  % box constraints on matrix variables
  penm.lbY = [0];
  penm.ubY = [Inf];

  penm.NgLIN=1;
  penm.lbg = [1];
  penm.ubg = [1];

  penm.objfun = @ncm1y_objfun;
  penm.objgrad = @ncm1y_objgrad;
  penm.objhess = @ncm1y_objhess;

  penm.confun = @ncm1y_confun;
  penm.congrad = @ncm1y_congrad;
  %penm.conhess = @sdp_conhess;  not needed because all linear


