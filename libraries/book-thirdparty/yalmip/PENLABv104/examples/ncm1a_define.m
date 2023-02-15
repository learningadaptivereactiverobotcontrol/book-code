function [penm] = ncm1a_define()
% define penm structure for Nearest Correlation Matrix Example 1
% using matrix inequalities A(x):
%    min_x sum (x_ij-h_ij)^2
%    s.t.  tr(X)=1
%          X >= 0 (pos. semidefinite)
%
  penm = [];

  penm.probname= 'NCM Example 1 using normal variables & matrix constraints';
  penm.comment = '3 diagonal matrix 3x3 as a vector of variables';

  % matrix H
  H = [2.2, -1.1, 0; -1.1, 1.9, -1.15; 0, -1.15, 2.1]./6;
  %H = [2.2, -1.1, 0; -1.1, 1.9, -1.1; 0, -1.1, 2.1]./6;
  %H = [2, -1, 0; -1, 2, -1; 0, -1, 2]./6;
  %H = [2.2, -1.1, 0; -1.1, 1.9, -1.1; 0, -1.1, 2.1];

  % matrix X will be expressed as:
  %   X = [ x_1, x_2,   0;
  %         x_2, x_3, x_4;
  %           0, x_4, x_5]  
  %
  %   XX = [ x(1), x(2), 0; x(2), x(3), x(4); 0, x(4), x(5)]  
  % by using matrices A{i} which just point where x(i) will be projected
  %   X = A{1}*x(1) + A{2}*x(2) + ... + A{5}*x(5)
  A = cell(5,1);
  A{1} = [1,0,0;
          0,0,0;
          0,0,0];
  A{2} = [0,1,0;
          1,0,0;
          0,0,0];
  A{3} = [0,0,0;
          0,1,0;
          0,0,0];
  A{4} = [0,0,0;
          0,0,1;
          0,1,0];
  A{5} = [0,0,0;
          0,0,0;
          0,0,1];

  userdata.H=H;
  userdata.A=A;

  % keep the whole structure
  penm.userdata=userdata;

  penm.Nx=5;

  % starting point
  penm.xinit=[1;0;1;0;1];

  penm.NgLIN=1;
  penm.lbg = [1];
  penm.ubg = [1];

  penm.NALIN=1;
  % and it should be positive semidefinite
  penm.lbA=[0];

  penm.objfun = @ncm1a_objfun;
  penm.objgrad = @ncm1a_objgrad;
  penm.objhess = @ncm1a_objhess;

  penm.confun = @(x,Y,userdata) deal(x(1)+x(3)+x(5), userdata);
  penm.congrad = @(x,Y,userdata) deal([1;0;1;0;1], userdata);
  %penm.confun = @ncm1a_confun;
  %penm.congrad = @ncm1a_congrad;
  %penm.conhess = @sdp_conhess;  not needed because all linear

  penm.mconfun = @ncm1a_mconfun;
  penm.mcongrad = @(x,Y,k,i,userdata) deal(sparse(userdata.A{i}), userdata);
  %penm.mcongrad = @ncm1a_mcongrad;
  %hessian not needed, function is linear


