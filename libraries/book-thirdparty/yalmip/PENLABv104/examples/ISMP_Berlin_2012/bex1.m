function [penm] = bex1()
% 
% call:
%   penm=ex3_define_by_anonymous();    % to define structure 
%   prob=penlab(penm);    % to convert the structure and initialize the problem
%   prob.opts....=...;    % to change option settings if desired
%   prob.solve();         % to start the solver
%   prob.x                % to retrieve the final point (solution)
%
% xopt = [2;0];

  B = sparse([1 -1 0; -1 1 0; 0 0 1]);
  A{1} = sparse([0 1 0; 1 0 0; 0 0 0]);
  A{2} = sparse([0 0 0; 0 0 1; 0 1 0]);

  penm = [];

  penm.probname = 'berlin_ex1';
  penm.comment = 'quad objective with LMI';

  penm.Nx=2;

  penm.NALIN=1;
  % let's make it positive semidefinite
  penm.lbA=zeros(1,1);

  % define all call-backs as anonymous functions
  penm.objfun  = @(x,Y,userdata) deal(-.5*(x(1)^2+x(2)^2), userdata);
  penm.objgrad = @(x,Y,userdata) deal(-[x(1);x(2)], userdata);
  penm.objhess = @(x,Y,userdata) deal(-eye(2,2), userdata);
  
  penm.mconfun  = @(x,Y,k,userdata) deal(B+A{1}*x(1)+A{2}*x(2), userdata);
  penm.mcongrad = @(x,Y,k,i,userdata) deal(A{i}, userdata);