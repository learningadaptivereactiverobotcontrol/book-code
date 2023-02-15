function [penm] = ex3_define_by_lagr()
% define Example 3 as is necessary to handle by PenLab
% same as ex3_define() but this time using Hessian of the Lagrangian
% call:
%   penm=ex3_define_by_lagr();    % to define structure 
%   prob=penlab(penm);    % to convert the structure and initialize the problem
%   prob.opts....=...;    % to change option settings if desired
%   prob.solve();         % to start the solver
%   prob.x                % to retrieve the final point (solution)

  penm = [];

  % [optional] set problem name/comment (for log files)
  penm.probname = 'examples/ex3';
  penm.comment = 'Source: user external definition of functions, using Hessian of the Lagrangian';

  penm.Nx = 2;
  penm.lbx = [-0.5; -Inf];
  penm.ubx = [0.5; 1];

  penm.NgNLN = 2;
  %penm.NgLIN = 0;          % optional, 0 by default
  penm.lbg = [0; 0];
  %penm.ubg = [Inf, Inf];   % optional, +Inf for upper bounds by default

  % possible even as anonymous?? Try it
  penm.objfun = @ex3_objfun;
  penm.confun = @ex3_confun;
  penm.objgrad = @ex3_objgrad;
  penm.congrad = @ex3_congrad;
  %penm.objhess = @ex3_objhess;
  %penm.conhess = @ex3_conhess;
  penm.lagrhess = @ex3_lagrhess;

  % [optional] set starting point
  %penm.x = zeros(2,1);
  penm.x = [-2; 1];        % suggested starting point


