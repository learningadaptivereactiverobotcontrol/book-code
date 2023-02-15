function [penm] = bex2()
% 
% call:
%   penm=ex3_define_by_anonymous();    % to define structure 
%   prob=penlab(penm);    % to convert the structure and initialize the problem
%   prob.opts....=...;    % to change option settings if desired
%   prob.solve();         % to start the solver
%   prob.x                % to retrieve the final point (solution)
%
% xopt = [-0.0000; 1.0384; 2.2271; 0.0000];

  B = sparse(4,4);
  A{1} = sparse([0 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 0]);
  A{2} = sparse([1 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 1]);
  A{3} = sparse([1 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 1]);
  A{4} = sparse([0 0 0 0; 0 2 0 0; 0 0 2 0; 0 0 0 0]);

  penm = [];

  penm.probname = 'berlin_ex2';
  penm.comment = 'quad objective with LMI';

  penm.Nx=4;

  penm.NgNLN = 3;
  penm.ubg = [0;0;0];
  
  penm.NALIN=1;
  % let's make it positive semidefinite
  penm.lbA=zeros(1,1);

  % define all call-backs as anonymous functions
  penm.objfun  = @(x,Y,userdata) deal(x(1)^2+x(2)^2+2*x(3)^2+x(4)^2-5*x(1)-5*x(2)-21*x(3)+7*x(4), userdata);
  penm.objgrad = @(x,Y,userdata) deal([2*x(1)-5;2*x(2)-5;4*x(3)-21;2*x(4)+7], userdata);
  penm.objhess = @(x,Y,userdata) deal([2 0 0 0;0 2 0 0; 0 0 4 0; 0 0 0 2], userdata);
  
  penm.confun = @(x,Y,userdata) deal([x(1)^2+x(2)^2+x(3)^2+x(4)^2+x(1)-x(2)+x(3)-x(4)-8;...
      x(1)^2+2*x(2)^2+x(3)^2+2*x(4)^2-x(1)-x(4)-10;...
      2*x(1)^2+x(2)^2+x(3)^2+2*x(1)-x(2)-x(4)-5], userdata);
  penm.congrad = @(x,Y,userdata) deal([2*x(1)+1, 2*x(2)-1, 2*x(3)+1, 2*x(4)-1;...
      2*x(1)-1, 4*x(2), 2*x(3), 4*x(4)-1;...
      4*x(1)+2, 2*x(2)-1, 2*x(3), -1]', userdata);
  %...except conhess...
  penm.conhess = @bex2_conhess;
  
  penm.mconfun  = @(x,Y,k,userdata) deal(A{1}*x(1)+A{2}*x(2)+A{3}*x(3)+A{4}*x(4), userdata);
  penm.mcongrad = @(x,Y,k,i,userdata) deal(A{i}, userdata);